#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2022 Stéphane Caron
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from typing import Any, Dict

import gin
import numpy as np
import pink
import pink.models
import pinocchio as pin
import upkie_description
from pink import solve_ik
from pink.tasks import BodyTask, PostureTask
from pink.utils import custom_configuration_vector

from agents.blue_balancer.wheel_balancer import WheelBalancer
from utils.clamp import clamp


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
        # tangent_index = configuration_index - 1
        # v[tangent_index] = observation["servo"][joint]["velocity"]
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


@gin.configurable
class WholeBodyController:

    """
    Coordinate inverse kinematics and wheel balancing.

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

    crouch_velocity: float
    gain_scale: float
    max_crouch_height: float
    robot: pin.RobotWrapper
    target_position_wheel_in_rest: np.ndarray
    tasks: Dict[str, Any]
    transform_rest_to_world: Dict[str, np.ndarray]
    turning_gain_scale: float

    def __init__(
        self,
        config: Dict[str, Any],
        crouch_velocity: float,
        gain_scale: float,
        max_crouch_height: float,
        turning_gain_scale: float,
    ):
        """
        Create controller.

        Args:
            config: Global configuration dictionary.
            crouch_velocity: Maximum vertical velocity in [m] / [s].
            gain_scale: PD gain scale for hip and knee joints.
            max_crouch_height: Maximum distance along the vertical axis that
                the robot goes down while crouching, in [m].
            turning_gain_scale: Additional gain scale added when the robot is
                turning to keep the legs stiff in spite of the ground pulling
                them apart.
        """
        robot = pink.models.build_from_urdf(upkie_description.urdf_path)
        configuration = pink.apply_configuration(robot, robot.q0)
        tasks = {
            "base": BodyTask(
                "base",
                position_cost=1.0,  # [cost] / [m]
                orientation_cost=1.0,  # [cost] / [rad]
            ),
            "left_contact": BodyTask(
                "left_contact",
                position_cost=[0.1, 0.0, 0.1],  # [cost] / [m]
                orientation_cost=0.0,  # [cost] / [rad]
                lm_damping=1e2,
            ),
            "right_contact": BodyTask(
                "right_contact",
                position_cost=[0.1, 0.0, 0.1],  # [cost] / [m]
                orientation_cost=0.0,  # [cost] / [rad]
                lm_damping=1e2,
            ),
            "posture": PostureTask(
                cost=1e-3,  # [cost] / [rad]
            ),
        }
        tasks["posture"].set_target(
            custom_configuration_vector(robot, left_knee=0.2, right_knee=-0.2)
        )
        self.__initialized = False
        self.configuration = configuration
        self.crouch_velocity = crouch_velocity
        self.gain_scale = clamp(gain_scale, 0.1, 2.0)
        self.max_crouch_height = max_crouch_height
        self.robot = robot
        self.servo_layout = config["servo_layout"]
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
        self.wheel_balancer = WheelBalancer()  # type: ignore
        self.turning_gain_scale = turning_gain_scale

    def update_target_height(
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
            velocity = self.crouch_velocity * axis_value
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
        self.update_target_height(observation, dt)
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
        self.configuration = pink.apply_configuration(self.robot, q)
        self.tasks["base"].set_target(
            self.configuration.get_transform_body_to_world("base")
        )
        for target in ["left_contact", "right_contact"]:
            transform_target_to_world = (
                self.configuration.get_transform_body_to_world(target)
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
        robot_velocity = solve_ik(self.configuration, self.tasks.values(), dt)
        q = self.configuration.integrate(robot_velocity, dt)
        self.configuration = pink.apply_configuration(self.robot, q)
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
        (
            left_wheel_velocity,
            right_wheel_velocity,
        ) = self.wheel_balancer.get_wheel_velocities(transform_right_to_left)

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
