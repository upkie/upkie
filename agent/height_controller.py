#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron
# Copyright 2023-2024 Inria

import gin
import meshcat_shapes
import numpy as np
import pink
import pinocchio as pin
import upkie_description
from numpy.typing import NDArray
from pink import solve_ik
from pink.tasks import FrameTask, PostureTask
from pink.utils import custom_configuration_vector
from pink.visualization import start_meshcat_visualizer
from upkie.utils.clamp import clamp
from upkie.utils.spdlog import logging

from .utils import abs_bounded_derivative_filter


def observe_configuration(
    observation, configuration, servo_layout
) -> NDArray[float]:
    """Compute configuration vector from a new observation.

    Args:
        observation: Observation dictionary.
        configuration: Previous configuration.
        servo_layout: Robot servo layout.

    Returns:
        Configuration vector from observation.
    """
    q = configuration.q.copy()
    for joint, servo in servo_layout.items():
        if "configuration_index" not in servo:
            continue
        i = servo["configuration_index"]
        q[i] = observation["servo"][joint]["position"]
    return q


def serialize_to_servo_action(configuration, velocity, servo_layout) -> dict:
    """Serialize robot state for the spine.

    Args:
        configuration: Robot configuration.
        velocity: Robot velocity in tangent space.
        servo_layout: Robot servo layout.

    Returns:
        Dictionary of position and velocity targets for each joint.
    """
    target = {}
    model = configuration.model
    tau_max = model.effortLimit
    for joint_name in servo_layout.keys():
        joint_id = model.getJointId(joint_name)
        joint = model.joints[joint_id]
        target[joint_name] = {
            "position": configuration.q[joint.idx_q],
            "velocity": velocity[joint.idx_v],
            "maximum_torque": tau_max[joint.idx_v],
        }
    return target


def add_target_frames(visualizer):
    """Add target frames for visualization.

    Args:
        visualizer: Meshcat viewer wrapper.
    """
    viewer = visualizer.viewer
    meshcat_shapes.frame(viewer["left_contact_target"], opacity=0.5)
    meshcat_shapes.frame(viewer["right_contact_target"], opacity=0.5)
    meshcat_shapes.frame(viewer["left_contact"], opacity=1.0)
    meshcat_shapes.frame(viewer["right_contact"], opacity=1.0)


@gin.configurable
class HeightController:
    """Compute leg inverse kinematics.

    Attributes:
        knees_forward: Set to True to bend knees forward rather than backward.
        max_crouch_height: Maximum distance along the vertical axis that the
            robot goes down while crouching, in [m].
        max_init_joint_velocity: Maximum joint velocity during the initial
            phase, in [rad] / [s].
        robot: Robot model used for inverse kinematics.
        target_position_wheel_in_rest: Target position in the rest frame.
        tasks: Dictionary of inverse kinematics tasks.
        transform_rest_to_world: Rest frame pose for each end effector.
    """

    height_difference: float = 0.0
    knees_forward: bool
    max_crouch_height: float
    max_crouch_velocity: float
    max_height_difference: float
    max_init_joint_velocity: float
    max_lean_velocity: float
    robot: pin.RobotWrapper
    target_height: float = 0.0
    target_position_wheel_in_rest: dict[NDArray[float]]
    tasks: dict
    transform_rest_to_world: dict

    def __init__(
        self,
        knees_forward: bool,
        max_crouch_height: float,
        max_crouch_velocity: float,
        max_height_difference: float,
        max_init_joint_velocity: float,
        max_lean_velocity: float,
        visualize: bool,
    ):
        """Create controller.

        Args:
            knees_forward: Set to True to bend knees forward rather than
                backward.
            max_crouch_height: Maximum distance along the vertical axis that
                the robot goes down while crouching, in [m].
            max_crouch_velocity: Maximum vertical velocity in [m] / [s].
            max_init_joint_velocity: Maximum joint velocity during the initial
                phase, in [rad] / [s].
            max_height_difference: Maximum height difference between the two
                wheel contact points, in [m].
            max_lean_velocity: Maximum leaning (to the side) velocity, in [m] /
                [s].
            visualize: If true, open a MeshCat visualizer on the side.
        """
        robot = upkie_description.load_in_pinocchio(root_joint=None)
        neutral_configuration = pink.Configuration(
            robot.model, robot.data, robot.q0
        )
        servo_layout = {
            "left_hip": {
                "bus": 2,
                "configuration_index": 0,
                "id": 4,
            },
            "left_knee": {
                "bus": 2,
                "configuration_index": 1,
                "id": 5,
            },
            "left_wheel": {
                "bus": 2,
                "id": 6,
            },
            "right_hip": {
                "bus": 1,
                "configuration_index": 3,
                "id": 1,
            },
            "right_knee": {
                "bus": 1,
                "configuration_index": 4,
                "id": 2,
            },
            "right_wheel": {
                "bus": 1,
                "id": 3,
            },
        }
        tasks = {
            "left_contact": FrameTask(
                "left_contact",
                position_cost=[0.1, 0.0, 0.1],  # [cost] / [m]
                orientation_cost=0.0,  # [cost] / [rad]
                lm_damping=10.0,
            ),
            "right_contact": FrameTask(
                "right_contact",
                position_cost=[0.1, 0.0, 0.1],  # [cost] / [m]
                orientation_cost=0.0,  # [cost] / [rad]
                lm_damping=10.0,
            ),
            "posture": PostureTask(
                cost=1e-3,  # [cost] / [rad]
            ),
        }

        sign = -1.0 if knees_forward else +1.0
        tasks["posture"].set_target(
            custom_configuration_vector(
                robot,
                left_hip=(-sign * 0.1),
                left_knee=(+sign * 0.2),
                right_hip=(+sign * 0.1),
                right_knee=(-sign * 0.2),
            )
        )

        transform_rest_to_world = {
            "left_contact": np.zeros((4, 4)),
            "right_contact": np.zeros((4, 4)),
        }
        for target in ["left_contact", "right_contact"]:
            transform_target_to_world = (
                neutral_configuration.get_transform_frame_to_world(target)
            )
            tasks[target].set_target(transform_target_to_world)
            transform_rest_to_world[target] = transform_target_to_world

        visualizer = None
        if visualize:
            visualizer = start_meshcat_visualizer(robot)
            add_target_frames(visualizer)

        logging.info("Initializing Upkie to its neutral configuration...")

        self.__initialized = False
        self.ik_configuration = neutral_configuration
        self.jump_playback = None
        self.knee_side = "forward" if knees_forward else "backward"
        self.last_velocity = np.zeros(robot.nv)
        self.max_crouch_height = max_crouch_height
        self.max_crouch_velocity = max_crouch_velocity
        self.max_height_difference = max_height_difference
        self.max_init_joint_velocity = max_init_joint_velocity
        self.max_lean_velocity = max_lean_velocity
        self.q_init = None
        self.robot = robot
        self.servo_layout = servo_layout
        self.target_position_wheel_in_rest = {
            k: np.zeros(3) for k in ("left_contact", "right_contact")
        }
        self.target_offset = {
            "left_contact": np.zeros(3),
            "right_contact": np.zeros(3),
        }
        self.tasks = tasks
        self.transform_rest_to_world = transform_rest_to_world
        self.visualizer = visualizer

    def get_next_height_from_joystick(
        self, observation: dict, dt: float
    ) -> float:
        """Update target base height from joystick inputs.

        Args:
            observation: Observation from the spine.
            dt: Duration in seconds until next cycle.

        Returns:
            New height target, in meters.
        """
        try:
            axis_value: float = observation["joystick"]["pad_axis"][1]
            velocity = self.max_crouch_velocity * axis_value
        except KeyError:
            velocity = 0.0

        height = self.target_height
        height += velocity * dt
        return height

    def get_next_height_difference_from_joystick(
        self, observation: dict, dt: float
    ):
        """Update the height difference from joystick inputs.

        Args:
            observation: Observation from the spine.
            dt: Duration in seconds until next cycle.

        Returns:
            New height difference, in meters.
        """
        try:
            axis_value: float = observation["joystick"]["pad_axis"][0]
            velocity = self.max_lean_velocity * axis_value
        except KeyError:
            velocity = 0.0
        delta = self.height_difference
        delta += velocity * dt
        return delta

    def update_target_height(self, observation: dict, dt: float) -> None:
        """Update target base height from joystick inputs.

        Args:
            observation: Observation from the spine.
            dt: Duration in seconds until next cycle.
        """
        height = self.get_next_height_from_joystick(observation, dt)
        self.target_height = clamp(height, 0.0, self.max_crouch_height)

        next_height_difference = self.get_next_height_difference_from_joystick(
            observation, dt
        )
        self.height_difference = clamp(
            next_height_difference,
            -self.max_height_difference,
            self.max_height_difference,
        )

        for wheel in ("left_contact", "right_contact"):
            offset = (
                self.height_difference
                if wheel == "right_contact"
                else -self.height_difference
            )
            offset /= 2

            # Clamp target heights
            self.target_position_wheel_in_rest[wheel][2] = clamp(
                self.target_height + offset, 0.0, self.max_crouch_height
            )

    def update_ik_targets(self, observation: dict, dt: float) -> None:
        """Update IK frame targets from individual target positions.

        Args:
            observation: Observation from the spine.
            dt: Duration in seconds until next cycle.
        """
        for target in ["left_contact", "right_contact"]:
            transform_common_to_rest = pin.SE3(
                rotation=np.eye(3),
                translation=self.target_position_wheel_in_rest[target],
            )
            transform_target_to_common = pin.SE3(
                rotation=np.eye(3),
                translation=self.target_offset[target],
            )
            transform_target_to_world = (
                self.transform_rest_to_world[target]
                * transform_common_to_rest
                * transform_target_to_common
            )
            self.tasks[target].set_target(transform_target_to_world)

    def _observe_ground_positions(self, observation: dict) -> None:
        """Observe the transform from right to left ground frames."""
        transform_left_to_world = self.tasks[
            "left_contact"
        ].transform_target_to_world
        transform_right_to_world = self.tasks[
            "right_contact"
        ].transform_target_to_world
        transform_right_to_left = transform_left_to_world.actInv(
            transform_right_to_world
        )
        observation["height_controller"] = {
            "position_right_in_left": transform_right_to_left.translation,
        }

        if self.visualizer is not None:
            self.visualizer.display(self.ik_configuration.q)
            viewer = self.visualizer.viewer
            viewer["left_contact_target"].set_transform(
                transform_left_to_world.np
            )
            viewer["right_contact_target"].set_transform(
                transform_right_to_world.np
            )

    def cycle(self, observation: dict, dt: float) -> dict:
        """Compute action for a new cycle.

        Args:
            observation: Latest observation.
            dt: Duration in seconds until next cycle.

        Returns:
            Dictionary with the new action and some internal state for logging.
        """
        servo_action = self.get_ik_servo_action(observation, dt)  # always run
        if not self.__initialized:
            servo_action = self.get_init_servo_action(observation, dt)
        action = {"servo": servo_action}
        return action

    def get_ik_servo_action(self, observation: dict, dt: float) -> dict:
        """Compute leg motion by differential inverse kinematics.

        Args:
            observation: Latest observation.
            dt: Duration in seconds until next cycle.

        Returns:
            Dictionary with the new action and some internal state for logging.
        """
        self.update_target_height(observation, dt)
        self.update_ik_targets(observation, dt)
        ik_velocity = solve_ik(
            self.ik_configuration,
            self.tasks.values(),
            dt,
            solver="proxqp",
        )
        self.ik_configuration.integrate_inplace(ik_velocity, dt)
        self.last_velocity = ik_velocity

        self._observe_ground_positions(observation)
        return serialize_to_servo_action(
            self.ik_configuration, ik_velocity, self.servo_layout
        )

    def get_init_servo_action(self, observation: dict, dt: float) -> dict:
        """Initial phase where we return legs to the neutral configuration.

        Args:
            observation: Observation from the spine.
            dt: Duration in seconds until next cycle.

        Returns:
            Dictionary with the new action.
        """
        if self.q_init is None:
            self.q_init = observe_configuration(
                observation, self.ik_configuration, self.servo_layout
            )
        # Applying this filter is ok as long as our root_joint is None
        self.q_init = abs_bounded_derivative_filter(
            prev_output=self.q_init,
            new_input=self.ik_configuration.q,
            dt=dt,
            max_derivative=np.full(
                (self.robot.nv,), self.max_init_joint_velocity
            ),
        )
        # Difference is also OK, configuration space is a vector space
        q_diff = self.q_init - self.ik_configuration.q
        if np.linalg.norm(q_diff, ord=1) < 1e-5:
            logging.info("Upkie initialized to the neutral configuration")
            self.__initialized = True
        return_configuration = pink.Configuration(
            self.robot.model, self.robot.data, self.q_init
        )
        return_velocity = np.zeros(self.robot.nv)
        return serialize_to_servo_action(
            return_configuration, return_velocity, self.servo_layout
        )
