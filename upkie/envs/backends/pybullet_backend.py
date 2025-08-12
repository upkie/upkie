#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

from typing import Optional

import numpy as np
import upkie_description

try:
    import pybullet
    import pybullet_data
except ModuleNotFoundError:
    pybullet = None
    pybullet_data = None

from upkie.config import BULLET_CONFIG
from upkie.exceptions import MissingOptionalDependency, UpkieRuntimeError
from upkie.model import Model
from upkie.utils.nested_update import nested_update
from upkie.utils.robot_state import RobotState
from upkie.utils.rotations import (
    quaternion_from_rotation_matrix,
    rotation_matrix_from_quaternion,
)

from .backend import Backend


class PyBulletBackend(Backend):
    r"""!
    Backend using PyBullet physics simulation.
    """

    def __init__(
        self,
        dt: float,
        bullet_config: Optional[dict] = None,
        gui: bool = True,
        nb_substeps: Optional[int] = None,
    ) -> None:
        r"""!
        Initialize PyBullet backend.

        \param dt Simulation time step in seconds.
        \param bullet_config Additional bullet configuration overriding the
            default `upkie.config.BULLET_CONFIG`. The combined configuration
            dictionary is used for PyBullet simulation setup.
        \param gui If True, run PyBullet with GUI. If False, run headless.
        \param nb_substeps Number of substeps for the PyBullet simulation.
        """
        # Combine dictionaries for simulator configuration
        self.__bullet_config = BULLET_CONFIG.copy()
        if bullet_config is not None:
            nested_update(self.__bullet_config, bullet_config)

        # Default number of substeps corresponds to the 1 kHz spine frequency
        nb_substeps: int = (
            nb_substeps if nb_substeps is not None else int(1000.0 * dt)
        )

        # Initialize PyBullet
        if pybullet is None or pybullet_data is None:
            raise MissingOptionalDependency(
                "PyBullet not found, "
                "you can install it by e.g. `pip install pybullet`"
            )
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
        pybullet.setTimeStep(dt / nb_substeps)

        # Load ground plane
        self._plane_id = pybullet.loadURDF("plane.urdf")

        # Load Upkie
        self._robot_id = pybullet.loadURDF(
            upkie_description.URDF_PATH,
            basePosition=[0, 0, 0.6],
            baseOrientation=[0, 0, 0, 1],
        )

        # Initialize model and build joint index mapping
        self.__model = Model(upkie_description.URDF_PATH)
        self._joint_indices = {}
        self._imu_link_index = -1
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

            link_name = joint_info[12].decode("utf-8")
            if link_name == "imu":
                self._imu_link_index = bullet_idx

        if self._imu_link_index < 0:
            raise UpkieRuntimeError("Robot does not have a link named 'imu'")

        # Initialize previous IMU velocity for acceleration computation
        self.__previous_imu_linear_velocity = np.zeros(3)

        if gui:  # Enable GUI if it is requested
            pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 1)
            pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_SHADOWS, 0)
            pybullet.resetDebugVisualizerCamera(
                cameraDistance=2.0,
                cameraYaw=45,
                cameraPitch=-30,
                cameraTargetPosition=[0, 0, 0.6],
            )

        # Internal attributes
        self.__dt = dt
        self.__nb_substeps = nb_substeps

    def __del__(self):
        """!
        Disconnect PyBullet when deleting the backend instance.
        """
        self.close()

    def close(self) -> None:
        """!
        Disconnect PyBullet properly.
        """
        if hasattr(self, "_bullet") and self._bullet is not None:
            pybullet.disconnect()
            self._bullet = None

    def reset(self, init_state: RobotState) -> dict:
        r"""!
        Reset the PyBullet simulation and get an initial observation.

        \param init_state Initial state of the robot (optional).
        \return Initial spine observation dictionary.
        """
        self.__reset_robot_state(init_state)
        pybullet.stepSimulation()
        return self.get_spine_observation()

    def __reset_robot_state(self, init_state: RobotState):
        # Reset base position and orientation
        position = init_state.position_base_in_world
        orientation_quat = init_state.orientation_base_in_world.as_quat()
        qx, qy, qz, qw = orientation_quat
        orientation_quat_bullet = [qx, qy, qz, qw]
        pybullet.resetBasePositionAndOrientation(
            self._robot_id,
            position,
            orientation_quat_bullet,
        )

        # Reset base velocity
        linear_velocity = init_state.linear_velocity_base_to_world_in_world
        angular_velocity = init_state.angular_velocity_base_in_base
        pybullet.resetBaseVelocity(
            self._robot_id,
            linear_velocity,
            angular_velocity,
        )

        # Reset joint states
        for joint in self.__model.joints:
            bullet_joint_idx = self._joint_indices[joint.name]
            pybullet.resetJointState(
                self._robot_id,
                bullet_joint_idx,
                init_state.joint_configuration[joint.idx_q],
            )

    def step(self, action: dict) -> dict:
        r"""!
        Apply action and step the PyBullet simulation.

        \param action Action dictionary in spine format.
        \return Spine observation dictionary after the step.
        """
        for _ in range(self.__nb_substeps):
            # Update motor torques at each substep
            servo_actions = action.get("servo", {})
            for joint_name, servo_action in servo_actions.items():
                if joint_name in self._joint_indices:
                    joint_idx = self._joint_indices[joint_name]
                    joint_torque: float = self.compute_joint_torque(
                        joint_name,
                        feedforward_torque=servo_action.get(
                            "feedforward_torque", 0.0
                        ),
                        target_position=servo_action["position"],
                        target_velocity=servo_action["velocity"],
                        kp_scale=servo_action.get("kp_scale", 1.0),
                        kd_scale=servo_action.get("kd_scale", 1.0),
                        maximum_torque=servo_action["maximum_torque"],
                    )
                    pybullet.setJointMotorControl2(
                        self._robot_id,
                        joint_idx,
                        pybullet.TORQUE_CONTROL,
                        force=joint_torque,
                    )

            # Step the simulation
            pybullet.stepSimulation()

        return self.get_spine_observation()

    def get_spine_observation(self) -> dict:
        """Get observation in spine format from PyBullet simulation."""
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
        position_base_in_world, bullet_quat_base_in_world = (
            pybullet.getBasePositionAndOrientation(self._robot_id)
        )
        (
            linear_velocity_base_to_world_in_world,
            angular_velocity_base_to_world_in_world,
        ) = pybullet.getBaseVelocity(self._robot_id)

        # Convert quaternion from Bullet format
        quat_base_to_world = [
            bullet_quat_base_in_world[3],  # w
            bullet_quat_base_in_world[0],  # x
            bullet_quat_base_in_world[1],  # y
            bullet_quat_base_in_world[2],  # z
        ]

        # Calculate pitch angle directly from quaternion
        qw, qx, qy, qz = quat_base_to_world
        pitch = np.arcsin(2 * (qw * qy - qz * qx))

        # Transform angular velocity from world frame to base frame
        rotation_base_to_world = rotation_matrix_from_quaternion(
            quat_base_to_world
        )
        rotation_world_to_base = rotation_base_to_world.T
        angular_velocity_base_in_base = rotation_world_to_base @ np.array(
            angular_velocity_base_to_world_in_world
        )

        return {
            "pitch": pitch,
            "angular_velocity": list(angular_velocity_base_in_base),
            "linear_velocity": list(linear_velocity_base_to_world_in_world),
        }

    def __get_imu_observation(self) -> dict:
        link_state = pybullet.getLinkState(
            self._robot_id,
            self._imu_link_index,
            computeLinkVelocity=True,
            computeForwardKinematics=True,
        )

        # Convert quaternion from Bullet format
        quat_imu_in_world = np.array(
            [
                link_state[5][3],  # w
                link_state[5][0],  # x
                link_state[5][1],  # y
                link_state[5][2],  # z
            ]
        )

        # The attitude reference system frame has +x forward, +y right and +z
        # down, whereas our world frame has +x forward, +y left and +z up:
        # https://github.com/mjbots/pi3hat/blob/master/docs/reference.md#orientation
        rotation_world_to_ars = np.diag([1.0, -1.0, -1.0])

        rotation_imu_to_world = rotation_matrix_from_quaternion(
            quat_imu_in_world
        )
        rotation_imu_to_ars = rotation_world_to_ars @ rotation_imu_to_world
        quat_imu_in_ars = quaternion_from_rotation_matrix(rotation_imu_to_ars)

        # Extract velocities
        linear_velocity_imu_in_world = np.array(link_state[6])
        angular_velocity_imu_to_world_in_world = np.array(link_state[7])

        # Compute linear acceleration in the world frame by discrete
        # differentiation
        linear_acceleration_imu_in_world = (
            linear_velocity_imu_in_world - self.__previous_imu_linear_velocity
        ) / self.__dt
        self.__previous_imu_linear_velocity = linear_velocity_imu_in_world

        # Transform angular velocity to IMU frame (C++ lines 68-70)
        rotation_world_to_imu = rotation_imu_to_world.T
        angular_velocity_imu_in_imu = (
            rotation_world_to_imu @ angular_velocity_imu_to_world_in_world
        )

        # Accelerometer readings (before and after filtering)
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
        position_base_in_world, _ = pybullet.getBasePositionAndOrientation(
            self._robot_id
        )
        contact = position_base_in_world[2] < 0.8
        return {
            "contact": contact,
        }

    def __get_servo_observations(self) -> dict:
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
        return servo_obs

    def __get_wheel_odometry_observation(self, servo_obs: dict) -> dict:
        left_wheel_pos = servo_obs["left_wheel"]["position"]
        right_wheel_pos = servo_obs["right_wheel"]["position"]
        left_wheel_vel = servo_obs["left_wheel"]["velocity"]
        right_wheel_vel = servo_obs["right_wheel"]["velocity"]

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

        # Use kp and kd gains from the bullet configuration
        torque_control_kp = self.__bullet_config["torque_control"]["kp"]
        torque_control_kd = self.__bullet_config["torque_control"]["kd"]
        kp = kp_scale * torque_control_kp
        kd = kd_scale * torque_control_kd

        # Compute joint torque with position-velocity feedback
        torque = feedforward_torque
        torque += kd * (target_velocity - measured_velocity)
        if not np.isnan(target_position):
            torque += kp * (target_position - measured_position)
        torque = np.clip(torque, -maximum_torque, maximum_torque)
        return torque
