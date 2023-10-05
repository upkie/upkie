#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2022 Stéphane Caron
# SPDX-License-Identifier: Apache-2.0

from typing import Any, Dict

import gin
import numpy as np
import pink
import pinocchio as pin
import upkie_description
from packaging import version
from pink import solve_ik
from pink.tasks import FrameTask, PostureTask
from pink.utils import custom_configuration_vector
from pink.visualization import start_meshcat_visualizer
from wheel_balancer import WheelBalancer

from upkie.utils.clamp import clamp

# Runtime version check for Pink for now
# See https://github.com/stephane-caron/bazel_pinocchio/issues/1 for context
MINIMUM_PINK_VERSION = "0.11.0"
if version.parse(pink.__version__) < version.parse(MINIMUM_PINK_VERSION):
    raise ImportError(
        f"Pink version {pink.__version__} is installed "
        f"but {MINIMUM_PINK_VERSION} is required"
    )


def observe(observation, configuration, servo_layout) -> np.ndarray:
    """
    Compute configuration vector from a new observation.

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


def serialize_to_servo_action(
    configuration, velocity, servo_layout
) -> Dict[str, dict]:
    """
    Serialize robot state for the spine.

    Args:
        configuration: Robot configuration.
        velocity: Robot velocity in tangent space.
        servo_layout: Robot servo layout.

    Returns:
        Dictionary of position and velocity targets for each joint.
    """
    target = {}
    for joint, servo in servo_layout.items():
        if "configuration_index" not in servo:
            continue
        i_q = servo["configuration_index"]
        i_v = i_q - 1
        target[joint] = {"position": configuration.q[i_q]}
        target[joint]["velocity"] = velocity[i_v]
    return target


def add_target_frames(visualizer):
    import meshcat_shapes

    viewer = visualizer.viewer
    meshcat_shapes.frame(viewer["left_contact_target"], opacity=0.5)
    meshcat_shapes.frame(viewer["right_contact_target"], opacity=0.5)
    meshcat_shapes.frame(viewer["left_contact"], opacity=1.0)
    meshcat_shapes.frame(viewer["right_contact"], opacity=1.0)


@gin.configurable
class WholeBodyController:

    """
    Coordinate leg inverse kinematics and wheel balancing.

    Attributes:
        crouch_velocity: Maximum vertical velocity in [m] / [s].
        gain_scale: PD gain scale for hip and knee joints.
        max_crouch_height: Maximum distance along the vertical axis that the
            robot goes down while crouching, in [m].
        robot: Robot model used for inverse kinematics.
        target_position_wheel_in_rest: Target position in the rest frame.
        tasks: Dictionary of inverse kinematics tasks.
        transform_rest_to_world: Rest frame pose for each end effector.
        turning_gain_scale: Additional gain scale added when the robot is
            turning to keep the legs stiff while the ground pulls them apart.
    """

    gain_scale: float
    max_crouch_height: float
    max_crouch_velocity: float
    robot: pin.RobotWrapper
    target_position_wheel_in_rest: np.ndarray
    tasks: Dict[str, Any]
    transform_rest_to_world: Dict[str, np.ndarray]
    turning_gain_scale: float

    def __init__(
        self,
        gain_scale: float,
        max_crouch_height: float,
        max_crouch_velocity: float,
        turning_gain_scale: float,
        visualize: bool,
    ):
        """
        Create controller.

        Args:
            gain_scale: PD gain scale for hip and knee joints.
            max_crouch_height: Maximum distance along the vertical axis that
                the robot goes down while crouching, in [m].
            max_crouch_velocity: Maximum vertical velocity in [m] / [s].
            turning_gain_scale: Additional gain scale added when the robot is
                turning to keep the legs stiff in spite of the ground pulling
                them apart.
            visualize: If true, open a MeshCat visualizer on the side.
        """
        robot = upkie_description.load_in_pinocchio(root_joint=None)
        configuration = pink.Configuration(robot.model, robot.data, robot.q0)
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
        tasks["posture"].set_target(
            custom_configuration_vector(robot, left_knee=0.2, right_knee=-0.2)
        )

        visualizer = None
        if visualize:
            visualizer = start_meshcat_visualizer(robot)
            add_target_frames(visualizer)

        self.__initialized = False
        self.configuration = configuration
        self.gain_scale = clamp(gain_scale, 0.1, 2.0)
        self.max_crouch_height = max_crouch_height
        self.max_crouch_velocity = max_crouch_velocity
        self.robot = robot
        self.servo_layout = servo_layout
        self.target_position_wheel_in_rest = np.zeros(3)
        self.target_offset = {
            "left_contact": np.zeros(3),
            "right_contact": np.zeros(3),
        }
        self.tasks = tasks
        self.transform_rest_to_world = {
            "left_contact": np.zeros((4, 4)),
            "right_contact": np.zeros((4, 4)),
        }
        self.turning_gain_scale = turning_gain_scale
        self.visualizer = visualizer
        self.wheel_balancer = WheelBalancer()  # type: ignore

    def update_target_heights(
        self, observation: Dict[str, Any], dt: float
    ) -> None:
        """
        Update target base height from joystick inputs.

        Args:
            observation: Observation from the spine.
            dt: Duration in seconds until next cycle.
        """
        try:
            axis_value: float = observation["joystick"]["pad_axis"][1]
            velocity = self.max_crouch_velocity * axis_value
        except KeyError:
            velocity = 0.0
        height = self.target_position_wheel_in_rest[2]
        height += velocity * dt
        self.target_position_wheel_in_rest[2] = clamp(
            height, 0.0, self.max_crouch_height
        )

    def update_ik_targets(
        self, observation: Dict[str, Any], dt: float
    ) -> None:
        """
        Update IK frame targets from individual target positions.

        Args:
            observation: Observation from the spine.
            dt: Duration in seconds until next cycle.
        """
        self.update_target_heights(observation, dt)
        transform_common_to_rest = pin.SE3(
            rotation=np.eye(3),
            translation=self.target_position_wheel_in_rest,
        )
        for target in ["left_contact", "right_contact"]:
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

    def _process_first_observation(self, observation: Dict[str, Any]) -> None:
        """
        Function called at the first iteration of the controller.

        Args:
            observation: Observation from the spine.
        """
        q = observe(observation, self.configuration, self.servo_layout)
        self.configuration = pink.Configuration(
            self.robot.model, self.robot.data, q
        )
        for target in ["left_contact", "right_contact"]:
            transform_target_to_world = (
                self.configuration.get_transform_frame_to_world(target)
            )
            self.tasks[target].set_target(transform_target_to_world)
            self.transform_rest_to_world[target] = transform_target_to_world
        self.__initialized = True

    def cycle(self, observation: Dict[str, Any], dt: float) -> Dict[str, Any]:
        """
        Compute action for a new cycle.

        Args:
            observation: Latest observation.
            dt: Duration in seconds until next cycle.

        Returns:
            Dictionary with the new action and some internal state for logging.
        """
        if not self.__initialized:
            self._process_first_observation(observation)

        self.update_ik_targets(observation, dt)
        robot_velocity = solve_ik(
            self.configuration, self.tasks.values(), dt, solver="quadprog"
        )
        self.configuration.integrate_inplace(robot_velocity, dt)
        servo_action = serialize_to_servo_action(
            self.configuration, robot_velocity, self.servo_layout
        )

        self.wheel_balancer.cycle(observation, dt)

        transform_left_to_world = self.tasks[
            "left_contact"
        ].transform_target_to_world
        transform_right_to_world = self.tasks[
            "right_contact"
        ].transform_target_to_world
        transform_right_to_left = transform_left_to_world.actInv(
            transform_right_to_world
        )
        position_right_in_left = transform_right_to_left.translation
        (
            left_wheel_velocity,
            right_wheel_velocity,
        ) = self.wheel_balancer.get_wheel_velocities(position_right_in_left)

        if self.visualizer is not None:
            self.visualizer.display(self.configuration.q)
            viewer = self.visualizer.viewer
            viewer["left_contact_target"].set_transform(
                transform_left_to_world.np
            )
            viewer["right_contact_target"].set_transform(
                transform_right_to_world.np
            )

        servo_action["left_wheel"] = {
            "position": np.nan,
            "velocity": left_wheel_velocity,
        }
        servo_action["right_wheel"] = {
            "position": np.nan,
            "velocity": right_wheel_velocity,
        }

        turning_prob = self.wheel_balancer.turning_probability
        # using the same numbers for both gain scales for now
        kp_scale = self.gain_scale + self.turning_gain_scale * turning_prob
        kd_scale = self.gain_scale + self.turning_gain_scale * turning_prob
        for joint_name in ["left_hip", "left_knee", "right_hip", "right_knee"]:
            servo_action[joint_name]["kp_scale"] = kp_scale
            servo_action[joint_name]["kd_scale"] = kd_scale

        return {
            "configuration": self.configuration.q,
            "servo": servo_action,
            "wheel_balancer": self.wheel_balancer.log(),
        }
