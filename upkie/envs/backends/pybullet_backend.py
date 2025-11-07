#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

## \namespace upkie.envs.backends.pybullet_backend
## \brief Backend using PyBullet physics simulation.

from typing import Dict, List, Optional

import numpy as np
import upkie_description

try:
    import pybullet
    import pybullet_data
except ModuleNotFoundError:
    ## PyBullet physics simulation library, or None if it is not installed.
    pybullet = None
    ## PyBullet data package, or None if PyBullet is not installed.
    pybullet_data = None

from upkie.config import BULLET_CONFIG, ROBOT_CONFIG
from upkie.exceptions import MissingOptionalDependency, UpkieRuntimeError
from upkie.model import Model
from upkie.utils.external_force import ExternalForce
from upkie.utils.nested_update import nested_update
from upkie.utils.point_contact import PointContact
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
        pybullet.loadURDF("plane.urdf")

        # Load Upkie
        self.__robot_id = pybullet.loadURDF(
            upkie_description.URDF_PATH,
            basePosition=[0, 0, 0.6],
            baseOrientation=[0, 0, 0, 1],
        )

        # Initialize model and build joint index mapping
        self.__link_index = {"base": -1}
        self.__link_name = {-1: "base"}
        self.__model = Model()
        self._joint_indices = {}
        self._joint_properties = {}
        for bullet_idx in range(pybullet.getNumJoints(self.__robot_id)):
            joint_info = pybullet.getJointInfo(self.__robot_id, bullet_idx)
            joint_name = joint_info[1].decode("utf-8")
            if joint_name in Model.JOINT_NAMES:
                self._joint_indices[joint_name] = bullet_idx
                # Initialize joint properties with defaults
                joint_props = self.__bullet_config.get(
                    "joint_properties", {}
                ).get(joint_name, {})
                self._joint_properties[joint_name] = {
                    "friction": joint_props.get("friction", 0.0),
                    "torque_control_noise": joint_props.get(
                        "torque_control_noise", 0.0
                    ),
                    "torque_measurement_noise": joint_props.get(
                        "torque_measurement_noise", 0.0
                    ),
                }
                # Disable velocity controllers to enable torque control
                pybullet.setJointMotorControl2(
                    self.__robot_id,
                    bullet_idx,
                    pybullet.VELOCITY_CONTROL,
                    force=0,
                )

            link_name = joint_info[12].decode("utf-8")
            self.__link_index[link_name] = bullet_idx
            self.__link_name[bullet_idx] = link_name

        if "imu" not in self.__link_index:
            raise UpkieRuntimeError("Robot does not have a link named 'imu'")

        # Initialize previous IMU velocity for acceleration computation
        self.__previous_imu_linear_velocity = np.zeros(3)

        # Initialize random number generator for noise simulation
        self.__rng = np.random.default_rng()

        # Initialize storage for commanded joint torques
        self.__joint_torques = {
            joint_name: 0.0 for joint_name in self._joint_indices.keys()
        }

        # Initialize storage for nominal masses and inertias for randomization
        self.__nominal_masses = {}
        self.__nominal_inertias = {}
        self.__save_nominal_inertias()

        # Apply inertia randomization if configured
        inertia_variation = self.__bullet_config.get("inertia_variation", 0.0)
        if abs(inertia_variation) > 1e-10:
            self.randomize_inertias(inertia_variation)

        if gui:  # Enable GUI if it is requested
            pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 1)
            pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_SHADOWS, 0)
            pybullet.resetDebugVisualizerCamera(
                cameraDistance=1.0,
                cameraYaw=45,
                cameraPitch=-30,
                cameraTargetPosition=[0, 0, 0.3],
            )

        # Internal attributes
        self.__dt = dt
        self.__external_forces = {}
        self.__nb_substeps = nb_substeps

    def __del__(self):
        """!
        Disconnect PyBullet when deleting the backend instance.
        """
        self.close()

    @property
    def robot_id(self) -> int:
        """!
        Identifier of the robot body in the PyBullet simulation.
        """
        return self.__robot_id

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

        \param init_state Initial state of the robot.
        \return Initial spine observation dictionary.
        """
        self._reset_robot_state(init_state)
        pybullet.stepSimulation()
        return self.get_spine_observation()

    def _reset_robot_state(self, init_state: RobotState):
        r"""!
        Reset robot to an initial state.

        \param init_state Initial state of the robot.
        """
        # Reset base position and orientation
        position = init_state.position_base_in_world
        orientation_quat = init_state.orientation_base_in_world.as_quat()
        qx, qy, qz, qw = orientation_quat
        orientation_quat_bullet = [qx, qy, qz, qw]
        pybullet.resetBasePositionAndOrientation(
            self.__robot_id,
            position,
            orientation_quat_bullet,
        )

        # Reset base velocity
        linear_velocity = init_state.linear_velocity_base_to_world_in_world
        angular_velocity = init_state.angular_velocity_base_in_base
        pybullet.resetBaseVelocity(
            self.__robot_id,
            linear_velocity,
            angular_velocity,
        )

        # Reset joint states
        for joint in self.__model.joints:
            bullet_joint_idx = self._joint_indices[joint.name]
            pybullet.resetJointState(
                self.__robot_id,
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
                    # Store the commanded torque for observation later
                    self.__joint_torques[joint_name] = joint_torque
                    pybullet.setJointMotorControl2(
                        self.__robot_id,
                        joint_idx,
                        pybullet.TORQUE_CONTROL,
                        force=joint_torque,
                    )

            # Apply external forces, if any
            self.__apply_external_forces()

            # Step the simulation
            pybullet.stepSimulation()

        return self.get_spine_observation()

    def get_spine_observation(self) -> dict:
        r"""!
        Get observation in spine format from PyBullet simulation.

        \return Spine observation dictionary.
        """
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
            pybullet.getBasePositionAndOrientation(self.__robot_id)
        )
        (
            linear_velocity_base_to_world_in_world,
            angular_velocity_base_to_world_in_world,
        ) = pybullet.getBaseVelocity(self.__robot_id)

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
            self.__robot_id,
            self.__link_index["imu"],
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
            self.__robot_id
        )
        contact = position_base_in_world[2] < 0.8
        return {
            "contact": contact,
        }

    def __get_servo_observations(self) -> dict:
        servo_obs = {}
        for joint_name, joint_idx in self._joint_indices.items():
            joint_state = pybullet.getJointState(
                self.__robot_id, joint_idx, physicsClientId=self._bullet
            )
            position = joint_state[0]  # in rad
            velocity = joint_state[1]  # in rad/s

            # Commanded torques are applied by the simulator exactly, so the
            # measured torque is the commanded one stored in __joint_torques
            torque = self.__joint_torques[joint_name]

            # Add Gaussian white noise to torque measurements if configured
            joint_props = self._joint_properties[joint_name]
            torque_measurement_noise = joint_props["torque_measurement_noise"]
            if torque_measurement_noise > 1e-10:
                noise = self.__rng.normal(0.0, torque_measurement_noise)
                torque += noise
            servo_obs[joint_name] = {
                "position": position,
                "velocity": velocity,
                "torque": torque,
                "temperature": 42.0,  # dummy value
                "voltage": 18.0,  # dummy value
            }
        return servo_obs

    def __get_wheel_odometry_observation(self, servo_obs: dict) -> dict:
        left_angle = servo_obs["left_wheel"]["position"]
        right_angle = servo_obs["right_wheel"]["position"]
        left_omega = servo_obs["left_wheel"]["velocity"]
        right_omega = servo_obs["right_wheel"]["velocity"]
        wheel_radius = ROBOT_CONFIG["wheel_radius"]

        ground_position = 0.5 * (left_angle - right_angle) * wheel_radius
        ground_velocity = 0.5 * (left_omega - right_omega) * wheel_radius

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
        \param feedforward_torque Feedforward torque command in N⋅m.
        \param target_position Target angular position in rad.
        \param target_velocity Target angular velocity in rad/s.
        \param kp_scale Multiplicative factor applied to the proportional gain
            in torque control.
        \param kd_scale Multiplicative factor applied to the derivative gain
            in torque control.
        \param maximum_torque Maximum torque in N·m from the command.
        \return Computed joint torque in N·m.

        This function should have the same semantics as \ref
        upkie::cpp::interfaces::BulletInterface::compute_joint_torque.
        """
        assert not np.isnan(target_velocity)

        # Read in measurements from the simulator
        joint_idx = self._joint_indices[joint_name]
        joint_state = pybullet.getJointState(self.__robot_id, joint_idx)
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

        # Add kinetic friction if applicable
        MAX_STICTION_VELOCITY = 1e-3  # rad/s
        if abs(measured_velocity) > MAX_STICTION_VELOCITY:
            friction = self._joint_properties[joint_name]["friction"]
            velocity_sign = 1.0 if measured_velocity > 0.0 else -1.0
            friction_torque = -friction * velocity_sign
            torque += friction_torque

        # Add torque-control Gaussian white noise if applicable
        joint_props = self._joint_properties[joint_name]
        torque_control_noise = joint_props["torque_control_noise"]
        if torque_control_noise > 1e-10:
            noise = self.__rng.normal(0.0, torque_control_noise)
            torque += noise

        torque = np.clip(torque, -maximum_torque, maximum_torque)
        return torque

    def __save_nominal_inertias(self) -> None:
        r"""!
        Save nominal masses and inertias for all robot links.

        This method stores the original dynamic properties of each link
        so they can be used later for randomization.
        """
        num_joints = pybullet.getNumJoints(self.__robot_id)
        for link_id in range(num_joints):
            dynamics_info = pybullet.getDynamicsInfo(self.__robot_id, link_id)
            mass = dynamics_info[0]
            local_inertia_diagonal = dynamics_info[2]  # tuple of 3 values

            self.__nominal_masses[link_id] = mass
            self.__nominal_inertias[link_id] = list(local_inertia_diagonal)

    def randomize_inertias(self, inertia_variation: float) -> None:
        r"""!
        Randomize the inertias of all robot links.

        \\param inertia_variation Magnitude of uniform noise to sample from.
            The actual variation is sampled uniformly from
            [-inertia_variation, +inertia_variation].

        This method applies random variations to both masses and inertias of
        all robot links. The same random factor (1 + epsilon) is applied to
        both mass and inertia components of each link, assuming uniform mass
        distribution.
        """
        for link_id in self.__nominal_masses:
            # Sample random variation uniformly from range
            epsilon = self.__rng.uniform(-inertia_variation, inertia_variation)

            # Calculate new mass and inertia values
            new_mass = self.__nominal_masses[link_id] * (1 + epsilon)
            new_inertia = [
                inertia_component * (1 + epsilon)
                for inertia_component in self.__nominal_inertias[link_id]
            ]

            # Apply the changes to the PyBullet simulation
            pybullet.changeDynamics(
                self.__robot_id,
                link_id,
                mass=new_mass,
                localInertiaDiagonal=new_inertia,
            )

    def set_external_forces(
        self, external_forces: Dict[str, ExternalForce]
    ) -> None:
        r"""!
        Set external forces to apply to robot links at next step.

        \param external_forces Dictionary specifying external forces to apply.
            Values must be ExternalForce instances.
        """
        for link_name, external_force in external_forces.items():
            link_index = self.__link_index.get(link_name)
            if link_index is None:
                raise UpkieRuntimeError(
                    f"Robot does not have a link named '{link_name}'"
                )

            # Store force specifications for application during simulation
            self.__external_forces[link_index] = {
                "force": external_force.force,
                "local": external_force.local,
            }

    def __apply_external_forces(self) -> None:
        r"""!
        Apply stored external forces to the robot in PyBullet.

        This is called internally during simulation steps to apply
        all forces that were previously set via @ref set_external_forces.
        """
        for link_index, force_spec in self.__external_forces.items():
            force = force_spec["force"]
            local_frame = force_spec["local"]

            if local_frame:
                # Force in local frame: apply at link origin
                position = [0.0, 0.0, 0.0]
                flags = pybullet.LINK_FRAME
            else:
                # Force in world frame: get link position in world
                if link_index == -1:
                    # Base link
                    position, _ = pybullet.getBasePositionAndOrientation(
                        self.__robot_id
                    )
                else:
                    # Other links
                    link_state = pybullet.getLinkState(
                        self.__robot_id, link_index
                    )
                    position = link_state[0]  # world position
                flags = pybullet.WORLD_FRAME

            # Apply the external force
            pybullet.applyExternalForce(
                self.__robot_id, link_index, force, position, flags
            )

    def get_contact_points(
        self, link_name: Optional[str] = None
    ) -> List[PointContact]:
        r"""!
        Get contact points from PyBullet simulation.

        \param link_name Optional link name to filter contacts. If provided,
            only returns contact points involving this specific link.
            If None, returns all contact points for the robot.
        \return List of PointContact instances representing contact points
            from the robot's perspective.
        """
        link_index = -2  # -2 for all links in PyBullet
        if link_name is not None:
            link_index = self.__link_index.get(link_name)
            if link_index is None:
                return []

        contact_points = pybullet.getContactPoints(
            bodyA=self.__robot_id,
            bodyB=-1,  # -1 for all bodies
            linkIndexA=link_index,
            linkIndexB=-2,  # -2 for all links
        )

        result = []
        for contact in contact_points:
            link_idx = contact[3]  # link index in body A
            contact_link_name = self.__link_name.get(
                link_idx, f"unknown_link_{link_idx}"
            )

            # Read contact properties returned by PyBullet
            position_contact_in_world = np.array(contact[5])  # A is robot link
            contact_normal = np.array(contact[7])  # from B to A
            normal_force_magnitude = contact[9]
            lateral_friction_1 = contact[10]
            lateral_friction_dir_1 = np.array(contact[11])
            lateral_friction_2 = contact[12]
            lateral_friction_dir_2 = np.array(contact[13])

            # Net contact force exerted by body B on body A
            normal_force = normal_force_magnitude * contact_normal
            friction_force_1 = lateral_friction_1 * lateral_friction_dir_1
            friction_force_2 = lateral_friction_2 * lateral_friction_dir_2
            contact_force = normal_force + friction_force_1 + friction_force_2

            # Append PointContact to the list
            contact_point = PointContact(
                force_in_world=contact_force,
                link_name=contact_link_name,
                position_contact_in_world=position_contact_in_world,
            )
            result.append(contact_point)

        return result
