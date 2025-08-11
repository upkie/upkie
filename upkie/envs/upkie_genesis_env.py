#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

from typing import Optional, Tuple

import numpy as np
import upkie_description

try:
    import genesis as genesis
except ModuleNotFoundError:
    genesis = None

from upkie.config import BULLET_CONFIG
from upkie.envs.pipelines import Pipeline
from upkie.exceptions import MissingOptionalDependency, UpkieRuntimeError
from upkie.utils.nested_update import nested_update
from upkie.utils.robot_state import RobotState
from upkie.utils.rotations import (
    quaternion_from_rotation_matrix,
    rotation_matrix_from_quaternion,
)

from .upkie_env import UpkieEnv


class UpkieGenesisEnv(UpkieEnv):
    """!
    Upkie environment using the
    [Genesis](https://genesis-embodied-ai.github.io/) physics simulator.
    """

    def __init__(
        self,
        backend: str = "gpu",
        frequency: Optional[float] = 200.0,
        frequency_checks: bool = False,
        genesis_config: Optional[dict] = None,
        gui: bool = True,
        init_state: Optional[RobotState] = None,
        pipeline: Optional[Pipeline] = None,
        regulate_frequency: bool = True,
    ) -> None:
        r"""!
        Initialize environment.

        \param backend Genesis backend to use ("cpu" or "gpu").
        \param frequency Regulated frequency of the control loop, in Hz. Can be
            prescribed even when `regulate_frequency` is unset, in which case
            `self.dt` will be defined but the loop frequency will not be
            regulated.
        \param frequency_checks If `regulate_frequency` is set and this
            parameter is True, a warning will be issued every time the control
            loop runs slower than the desired `frequency`. This check is
            disabled by default in this environment as the simulation step is
            executed inside the control loop.
        \param genesis_config Additional Genesis configuration overriding the
            default configuration. The combined configuration dictionary is
            used for Genesis simulation setup.
        \param gui If True, run Genesis with GUI. If False, run headless.
        \param init_state Initial state of the robot, only used in simulation.
        \param pipeline Spine dictionary interface selected via the --pipeline
            command-line argument of the spine binary, if any.
        \param regulate_frequency If set (default), the environment will
            regulate the control loop frequency to the value prescribed in
            `frequency`.
        """
        super().__init__(
            frequency=frequency,
            frequency_checks=frequency_checks,
            init_state=init_state,
            pipeline=pipeline,
            regulate_frequency=regulate_frequency,
        )

        # Save combined simulator configuration
        self.__genesis_config = (
            BULLET_CONFIG.copy()
        )  # Reuse bullet config structure
        if genesis_config is not None:
            nested_update(self.__genesis_config, genesis_config)

        # Initialize Genesis
        if genesis is None:
            raise MissingOptionalDependency(
                "Genesis not found, "
                "you can install it e.g. by `pip install genesis-world`"
            )
        if backend == "gpu":
            genesis.init(backend=genesis.gpu)
        elif backend == "cpu":
            genesis.init(backend=genesis.cpu)
        else:
            raise UpkieRuntimeError(f"Unknown Genesis backend: {backend}")

        # Create scene
        self._scene = genesis.Scene(
            viewer_options=(
                genesis.options.ViewerOptions(
                    camera_pos=[2.0, 0.0, 1.2],
                    camera_lookat=[0.0, 0.0, 0.6],
                    camera_up=[0.0, 0.0, 1.0],
                    max_FPS=60,
                )
                if gui
                else None
            ),
            sim_options=genesis.options.SimOptions(
                dt=self.dt,
                substeps=1,
                gravity=[0.0, 0.0, -9.81],
            ),
            profiling_options=genesis.options.ProfilingOptions(
                show_FPS=True,
            ),
            show_viewer=gui,
        )

        # Add entities to the scene
        self._plane = self._scene.add_entity(genesis.morphs.Plane())
        self._robot = self._scene.add_entity(
            genesis.morphs.URDF(
                file=upkie_description.URDF_PATH, pos=[0, 0, 0.6]
            )
        )

        # List of joint indices in the robot entity
        self._joint_indices = {}
        for joint in self.model.joints:
            joint = self._robot.get_joint(joint.name)
            self._joint_indices[joint.name] = int(*joint.dofs_idx_local)

        # Other attributes
        self.__previous_imu_linear_velocity = np.zeros(3)

    def __del__(self):
        """!
        Clean up Genesis when deleting the environment instance.
        """
        self.close()

    def close(self) -> None:
        """!
        Clean up Genesis properly.
        """
        if hasattr(self, "_scene") and self._scene is not None:
            self._scene = None

    def reset(
        self,
        seed: Optional[int] = None,
        options: Optional[dict] = None,
    ) -> Tuple[dict, dict]:
        r"""!
        Resets the Genesis simulation and get an initial observation.

        \param seed Number used to initialize the environment's internal random
            number generator.
        \param options Currently unused.
        \return
            - `observation`: Initial vectorized observation, i.e. an element
              of the environment's `observation_space`.
            - `info`: Dictionary with auxiliary diagnostic information. For
              Upkie this is the full observation dictionary sent by the spine.
        """
        super().reset(seed=seed)

        # Build the scene if it was not already built
        if not hasattr(self._scene, "_built") or not self._scene._built:
            self._scene.build()

        self._reset_robot_state()
        self._scene.step()
        spine_observation = self._get_spine_observation()
        observation = self.get_env_observation(spine_observation)
        info = {"spine_observation": spine_observation}
        return observation, info

    def _reset_robot_state(self):
        """Reset robot to initial state with randomization."""
        init_state, np_random = self.init_state, self.np_random

        # Reset base pose in the world frame
        position = init_state.sample_position(np_random)
        orientation_matrix = init_state.sample_orientation(np_random)
        qx, qy, qz, qw = orientation_matrix.as_quat()
        self._robot.set_pos(position)
        self._robot.set_quat([qw, qx, qy, qz])

        # Missing: Reset base velocity in the world frame
        # NB: the Genesis API does not do base velocity reset as of 0.2.1

        # Reset joint states
        dofs_position = []
        dofs_velocity = []
        dofs_idx = []
        for joint in self.model.joints:
            dofs_idx.append(self._joint_indices[joint.name])
            dofs_position.append(init_state.joint_configuration[joint.idx_q])
            dofs_velocity.append(init_state.joint_velocity[joint.idx_v])
        self._robot.set_dofs_position(dofs_position, dofs_idx_local=dofs_idx)
        self._robot.set_dofs_velocity(dofs_velocity, dofs_idx_local=dofs_idx)

    def step(self, action: dict) -> Tuple[dict, float, bool, bool, dict]:
        r"""!
        Run one timestep of the environment's dynamics.

        When the end of the episode is reached, you are responsible for calling
        `reset()` to reset the environment's state.

        \param action Action from the agent.
        \return
            - `observation`: Observation of the environment, i.e. an element
              of its `observation_space`.
            - `reward`: Reward returned after taking the action.
            - `terminated`: Whether the agent reached a terminal state,
              which may be a good or a bad thing. When true, the user needs to
              call `reset()`.
            - `truncated`: Whether the episode is reaching max number of
              steps. This boolean can signal a premature end of the episode,
              i.e. before a terminal state is reached. When true, the user
              needs to call `reset()`.
            - `info`: Dictionary with additional information, reporting in
              particular the full observation dictionary coming from the spine.
        """
        # Regulate loop frequency, if applicable
        super().step()

        # Apply motor torques
        joint_torques = []
        dofs_idx_local = []
        for joint_name, servo_action in action.items():
            joint_torque = self.compute_joint_torque(
                joint_name,
                feedforward_torque=servo_action.get("feedforward_torque", 0.0),
                target_position=servo_action["position"],
                target_velocity=servo_action["velocity"],
                kp_scale=servo_action.get("kp_scale", 1.0),
                kd_scale=servo_action.get("kd_scale", 1.0),
                maximum_torque=servo_action["maximum_torque"],
            )
            dofs_idx_local.append(self._joint_indices[joint_name])
            joint_torques.append(joint_torque)
        self._robot.control_dofs_force(
            joint_torques,
            dofs_idx_local=dofs_idx_local,
        )

        # Step the simulation
        self._scene.step()

        # Get observation
        spine_observation = self._get_spine_observation()
        observation = self.get_env_observation(spine_observation)
        reward = 1.0  # reward can be decided by a wrapper
        terminated = False
        truncated = False  # will be handled by e.g. a TimeLimit wrapper
        info = {"spine_observation": spine_observation}

        return observation, reward, terminated, truncated, info

    def _get_spine_observation(self) -> dict:
        """Get observation in spine format from Genesis simulation."""
        base_orientation = self.__get_base_orientation_observation()
        imu = self.__get_imu_observation()
        floor_contact = self.__get_floor_contact_observation()
        servo_obs = self.__get_servo_observations()
        wheel_odometry = self.__get_wheel_odometry_observation(servo_obs)

        return {
            "base_orientation": base_orientation,
            "floor_contact": floor_contact,
            "imu": imu,
            "servo": servo_obs,
            "wheel_odometry": wheel_odometry,
        }

    def __get_base_orientation_observation(self) -> dict:
        quaternion = self._robot.get_quat().cpu().numpy()  # [w, x, y, z]
        linear_velocity = self._robot.get_vel().cpu().numpy()
        angular_velocity = self._robot.get_ang().cpu().numpy()

        # Convert quaternion to our format [w, x, y, z]
        quat_base_to_world = quaternion

        # Calculate pitch angle directly from quaternion
        qw, qx, qy, qz = quat_base_to_world
        pitch = np.arcsin(2 * (qw * qy - qz * qx))

        # Transform angular velocity from world frame to base frame
        rotation_base_to_world = rotation_matrix_from_quaternion(
            quat_base_to_world
        )
        rotation_world_to_base = rotation_base_to_world.T
        angular_velocity_base_in_base = (
            rotation_world_to_base @ angular_velocity
        )

        return {
            "pitch": pitch,
            "angular_velocity": list(angular_velocity_base_in_base),
            "linear_velocity": list(linear_velocity),
        }

    def __get_imu_observation(self) -> dict:
        # Get IMU link state (this API might vary in Genesis)
        quat_base_to_world = self._robot.get_quat().cpu().numpy()
        linear_velocity_base_to_world_in_world = (
            self._robot.get_vel().cpu().numpy()
        )
        angular_velocity_base_to_world_in_world = (
            self._robot.get_ang().cpu().numpy()
        )

        # The attitude reference system frame has +x forward, +y right and +z
        # down, whereas our world frame has +x forward, +y left and +z up
        rotation_world_to_ars = np.diag([1.0, -1.0, -1.0])

        rotation_base_to_world = rotation_matrix_from_quaternion(
            quat_base_to_world
        )
        rotation_imu_to_base = self.model.rotation_base_to_imu.T
        rotation_imu_to_world = rotation_base_to_world @ rotation_imu_to_base
        rotation_imu_to_ars = rotation_world_to_ars @ rotation_imu_to_world
        quat_imu_in_ars = quaternion_from_rotation_matrix(rotation_imu_to_ars)

        # Compute linear acceleration by discrete differentiation
        linear_velocity_imu = linear_velocity_base_to_world_in_world
        linear_acceleration_imu_in_world = (
            linear_velocity_imu - self.__previous_imu_linear_velocity
        ) / self.dt
        self.__previous_imu_linear_velocity = linear_velocity_imu

        # Transform angular velocity to IMU frame
        rotation_world_to_imu = rotation_imu_to_world.T
        angular_velocity_imu_in_imu = (
            rotation_world_to_imu @ angular_velocity_base_to_world_in_world
        )

        # Accelerometer readings
        linear_acceleration_imu_in_imu = (
            rotation_world_to_imu @ linear_acceleration_imu_in_world
        )
        gravity_in_world = np.array([0.0, 0.0, -9.81])
        proper_acceleration_in_imu = rotation_world_to_imu @ (
            linear_acceleration_imu_in_world - gravity_in_world
        )

        return {
            "orientation": list(quat_imu_in_ars),
            "angular_velocity": list(angular_velocity_imu_in_imu),
            "linear_acceleration": linear_acceleration_imu_in_imu,
            "raw_linear_acceleration": list(proper_acceleration_in_imu),
        }

    def __get_floor_contact_observation(self) -> dict:
        position = self._robot.get_pos().cpu().numpy()
        contact = position[2] < 0.8
        return {
            "contact": contact,
        }

    def __get_servo_observations(self) -> dict:
        servo_obs = {}
        for joint_name, joint_idx in self._joint_indices.items():
            # Get joint state from Genesis
            position = (
                self._robot.get_dofs_position(dofs_idx_local=[joint_idx])
                .cpu()
                .numpy()[0]
            )
            velocity = (
                self._robot.get_dofs_velocity(dofs_idx_local=[joint_idx])
                .cpu()
                .numpy()[0]
            )
            torque = (
                self._robot.get_dofs_force(dofs_idx_local=[joint_idx])
                .cpu()
                .numpy()[0]
            )

            servo_obs[joint_name] = {
                "position": position,
                "velocity": velocity,
                "torque": torque,
                "temperature": 42.0,  # dummy value
                "voltage": 18.0,  # dummy value
            }
        return servo_obs

    def __get_wheel_odometry_observation(self, servo_obs: dict) -> dict:
        left_wheel_pos = servo_obs.get("left_wheel", {}).get("position", 0.0)
        right_wheel_pos = servo_obs.get("right_wheel", {}).get("position", 0.0)
        left_wheel_vel = servo_obs.get("left_wheel", {}).get("velocity", 0.0)
        right_wheel_vel = servo_obs.get("right_wheel", {}).get("velocity", 0.0)

        wheel_radius = 0.06  # approximate wheel radius in meters
        ground_position = (
            0.5 * (left_wheel_pos - right_wheel_pos) * wheel_radius
        )
        ground_velocity = (
            0.5 * (left_wheel_vel - right_wheel_vel) * wheel_radius
        )

        return {
            "position": ground_position,
            "velocity": ground_velocity,
        }

    def compute_joint_torque(
        self,
        joint_name: str,
        feedforward_torque: float,
        target_position: float,
        target_velocity: float,
        kp_scale: float,
        kd_scale: float,
        maximum_torque: float,
    ) -> float:
        r"""!
        Reproduce the moteus position controller in Genesis.

        \param joint_name Name of the joint.
        \param feedforward_torque Feedforward torque command in [N m].
        \param target_position Target angular position in [rad].
        \param target_velocity Target angular velocity in [rad] / [s].
        \param kp_scale Multiplicative factor applied to the proportional gain
            in torque control.
        \param kd_scale Multiplicative factor applied to the derivative gain
            in torque control.
        \param maximum_torque Maximum torque in [N m] from the command.
        \return Computed joint torque in [N m].
        """
        assert not np.isnan(target_velocity)

        # Read measurements from Genesis
        joint_idx = self._joint_indices[joint_name]
        measured_position = (
            self._robot.get_dofs_position(dofs_idx_local=[joint_idx])
            .cpu()
            .numpy()[0]
        )
        measured_velocity = (
            self._robot.get_dofs_velocity(dofs_idx_local=[joint_idx])
            .cpu()
            .numpy()[0]
        )

        # Use kp and kd gains from the configuration
        torque_control_kp = self.__genesis_config["torque_control"]["kp"]
        torque_control_kd = self.__genesis_config["torque_control"]["kd"]
        kp = kp_scale * torque_control_kp
        kd = kd_scale * torque_control_kd

        # Compute joint torque with position-velocity feedback
        torque = feedforward_torque
        torque += kd * (target_velocity - measured_velocity)
        if not np.isnan(target_position):
            torque += kp * (target_position - measured_position)
        torque = np.clip(torque, -maximum_torque, maximum_torque)
        return torque
