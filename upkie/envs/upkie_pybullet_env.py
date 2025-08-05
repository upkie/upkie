#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2025 Inria

from typing import Optional, Tuple

import numpy as np
import pybullet
import pybullet_data
import upkie_description

from upkie.config import SPINE_CONFIG
from upkie.envs.pipelines import Pipeline
from upkie.model import Model
from upkie.utils.robot_state import RobotState

from .upkie_env import UpkieEnv


class UpkiePyBulletEnv(UpkieEnv):
    r"""!
    Upkie environment using PyBullet physics simulation.
    """

    def __init__(
        self,
        frequency: Optional[float] = 200.0,
        frequency_checks: bool = True,
        gui: bool = False,
        init_state: Optional[RobotState] = None,
        pipeline: Optional[Pipeline] = None,
        regulate_frequency: bool = True,
    ) -> None:
        r"""!
        Initialize environment.

        \param frequency Regulated frequency of the control loop, in Hz. Can be
            prescribed even when `regulate_frequency` is unset, in which case
            `self.dt` will be defined but the loop frequency will not be
            regulated.
        \param frequency_checks If `regulate_frequency` is set and this
            parameter is true (default), a warning is issued every time the
            control loop runs slower than the desired `frequency`. Set this
            parameter to false to disable these warnings.
        \param gui If True, run PyBullet with GUI. If False, run headless.
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

        # Initialize PyBullet
        pybullet_mode = pybullet.GUI if gui else pybullet.DIRECT
        self._bullet = pybullet.connect(pybullet_mode)

        # Disable scene during initialization
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 0)
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 0)
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_SHADOWS, 0)

        # Set up simulator parameters
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
        pybullet.setGravity(0, 0, -9.81)
        pybullet.setRealTimeSimulation(False)  # making sure
        pybullet.setTimeStep(self.dt)

        # Load ground plane
        self._plane_id = pybullet.loadURDF("plane.urdf")

        # Load Upkie
        self._robot_id = pybullet.loadURDF(
            upkie_description.URDF_PATH,
            basePosition=[0, 0, 0.6],
            baseOrientation=[0, 0, 0, 1],
        )

        # Build joint index mapping
        self._joint_indices = {}
        for bullet_idx in range(pybullet.getNumJoints(self._robot_id)):
            joint_info = pybullet.getJointInfo(self._robot_id, bullet_idx)
            joint_name = joint_info[1].decode("utf-8")
            if joint_name in Model.JOINT_NAMES:
                self._joint_indices[joint_name] = bullet_idx
                # Disable velocity controllers to enable torque control
                pybullet.setJointMotorControl2(
                    self._robot_id,
                    bullet_idx,
                    pybullet.VELOCITY_CONTROL,
                    force=0,
                )

        if gui:  # Enable GUI if it is request
            pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 1)
            pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_SHADOWS, 0)
            pybullet.resetDebugVisualizerCamera(
                cameraDistance=2.0,
                cameraYaw=45,
                cameraPitch=-30,
                cameraTargetPosition=[0, 0, 0.6],
            )

    def __del__(self):
        """!
        Disconnect PyBullet when deleting the environment instance.
        """
        self.close()

    def close(self) -> None:
        """!
        Disconnect PyBullet properly.
        """
        if self._bullet is not None:
            pybullet.disconnect()
            self._bullet = None

    def reset(
        self,
        seed: Optional[int] = None,
        options: Optional[dict] = None,
    ) -> Tuple[dict, dict]:
        r"""!
        Resets the PyBullet simulation and get an initial observation.

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
        self._reset_robot_state()
        pybullet.stepSimulation()
        spine_observation = self._get_spine_observation()
        observation = self.get_env_observation(spine_observation)
        info = {"spine_observation": spine_observation}
        return observation, info

    def _reset_robot_state(self):
        """Reset robot to initial state with randomization."""
        init_state, np_random = self.init_state, self.np_random

        # Sample initial position and orientation
        position = init_state.sample_position(np_random)
        orientation_matrix = init_state.sample_orientation(np_random)
        qx, qy, qz, qw = orientation_matrix.as_quat()
        orientation_quat = [qx, qy, qz, qw]

        # Sample initial velocities
        linear_velocity = init_state.sample_linear_velocity(np_random)
        angular_velocity = init_state.sample_angular_velocity(np_random)

        # Reset base pose and velocity
        pybullet.resetBasePositionAndOrientation(
            self._robot_id,
            position,
            orientation_quat,
        )
        pybullet.resetBaseVelocity(
            self._robot_id,
            linear_velocity,
            angular_velocity,
        )

        # Reset joint states
        for joint in self.model.joints:
            bullet_joint_idx = self._joint_indices[joint.name]
            pybullet.resetJointState(
                self._robot_id,
                bullet_joint_idx,
                init_state.joint_configuration[joint.idx_q],
            )

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

        # Set motor torques for all joints
        for joint_name, servo_action in action.items():
            if joint_name in self._joint_indices:
                joint_idx = self._joint_indices[joint_name]
                target_position: float = servo_action["position"]
                target_velocity: float = servo_action["velocity"]
                feedforward_torque: float = servo_action.get(
                    "feedforward_torque", 0.0
                )
                kp_scale: float = servo_action.get("kp_scale", 1.0)
                kd_scale: float = servo_action.get("kd_scale", 1.0)
                maximum_torque: float = servo_action["maximum_torque"]
                joint_torque: float = self.compute_joint_torque(
                    joint_name,
                    feedforward_torque,
                    target_position,
                    target_velocity,
                    kp_scale,
                    kd_scale,
                    maximum_torque,
                )
                pybullet.setJointMotorControl2(
                    self._robot_id,
                    joint_idx,
                    pybullet.TORQUE_CONTROL,
                    force=joint_torque,
                )

        # Step the simulation
        pybullet.stepSimulation()

        # Get observation
        spine_observation = self._get_spine_observation()
        observation = self.get_env_observation(spine_observation)
        reward = 1.0  # reward can be decided by a wrapper
        terminated = False
        truncated = False  # will be handled by e.g. a TimeLimit wrapper
        info = {"spine_observation": spine_observation}

        return observation, reward, terminated, truncated, info

    def _get_spine_observation(self) -> dict:
        """Get observation in spine format from PyBullet simulation."""
        # Get base state
        base_pos, base_ori = pybullet.getBasePositionAndOrientation(
            self._robot_id
        )
        base_lin_vel, base_ang_vel = pybullet.getBaseVelocity(self._robot_id)

        # Calculate pitch from quaternion
        base_ori_wxyz = [base_ori[3], base_ori[0], base_ori[1], base_ori[2]]
        qw, qx, qy, qz = base_ori_wxyz
        pitch = np.arcsin(2 * (qw * qy - qz * qx))

        # Check floor contact (simplified - check if base is close to ground)
        contact = base_pos[2] < 0.8  # approximate contact detection

        # Get joint states
        servo_obs = {}
        for joint_name, joint_idx in self._joint_indices.items():
            joint_state = pybullet.getJointState(
                self._robot_id, joint_idx, physicsClientId=self._bullet
            )
            position = joint_state[0]  # in [rad]
            velocity = joint_state[1]  # in [rad] / [s]
            torque = joint_state[3]  # in [N m]
            servo_obs[joint_name] = {
                "position": position,
                "velocity": velocity,
                "torque": torque,
                "temperature": 42.0,  # dummy value
                "voltage": 18.0,  # dummy value
            }

        # Calculate wheel odometry (simplified)
        left_wheel_pos = servo_obs.get("left_wheel", {}).get("position", 0.0)
        right_wheel_pos = servo_obs.get("right_wheel", {}).get("position", 0.0)
        wheel_radius = 0.06  # approximate wheel radius in meters
        wheel_position = (
            (left_wheel_pos + right_wheel_pos) * 0.5 * wheel_radius
        )

        left_wheel_vel = servo_obs.get("left_wheel", {}).get("velocity", 0.0)
        right_wheel_vel = servo_obs.get("right_wheel", {}).get("velocity", 0.0)
        wheel_velocity = (
            (left_wheel_vel + right_wheel_vel) * 0.5 * wheel_radius
        )

        spine_observation = {
            "base_orientation": {
                "pitch": pitch,
                "angular_velocity": list(base_ang_vel),
                "linear_velocity": list(base_lin_vel),
            },
            "floor_contact": {
                "contact": contact,
            },
            "imu": {
                "orientation": base_ori_wxyz,
                "angular_velocity": list(base_ang_vel),
                "linear_acceleration": [0.0, 0.0, -9.81],  # dummy vector
            },
            "number": 0,
            "servo": servo_obs,
            "wheel_odometry": {
                "position": wheel_position,
                "velocity": wheel_velocity,
            },
        }

        return spine_observation

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
        r"""!\
        Reproduce the moteus position controller in PyBullet.

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

        This function should have the same semantics as \ref
        upkie::cpp::interfaces::BulletInterface::compute_joint_torque.
        """
        assert not np.isnan(target_velocity)

        # Read in measurements from the simulator
        joint_idx = self._joint_indices[joint_name]
        joint_state = pybullet.getJointState(self._robot_id, joint_idx)
        measured_position = joint_state[0]  # already in radians in PyBullet
        measured_velocity = joint_state[1]  # already in rad/s in PyBullet

        # Use kp and kd gains from the global spine configuration
        torque_control_kp = SPINE_CONFIG["bullet"]["torque_control"]["kp"]
        torque_control_kd = SPINE_CONFIG["bullet"]["torque_control"]["kd"]
        kp = kp_scale * torque_control_kp
        kd = kd_scale * torque_control_kd

        # Compute joint torque with position-velocity feedback
        torque = feedforward_torque
        torque += kd * (target_velocity - measured_velocity)
        if not np.isnan(target_position):
            torque += kp * (target_position - measured_position)
        torque = np.clip(torque, -maximum_torque, maximum_torque)
        return torque
