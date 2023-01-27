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

from agents.blue_balancer.kinematics import (
    forward_kinematics,
    velocity_limited_inverse_kinematics,
    velocity_limited_joint_control,
)
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


@gin.configurable
class WholeBodyController:

    """
    Coordinate inverse kinematics and wheel balancing.

    Attributes:
        crouch_height: Target vertical distance in meters by which to
            crouch (equivalently: lift the wheels), with respect to the initial
            configuration where the legs are extended. The target height (e.g.
            of the COM) above ground is therefore equal to ``maximum_height -
            crouch_height``.
        crouch_velocity: Desired time derivative for the target crouch
            height, in m / s.
        gain_scale: PD gain scale for hip and knee joints.
        max_crouch_height: Maximum distance along the vertical axis that the
            robot goes down while crouching, in meters.
        max_crouch_velocity: Maximum vertical velocity in m / s.
        position_right_in_left: Translation from the left contact frame to
            the right contact frame, expressed in the left contact frame.
        turning_gain_scale: Additional gain scale added when the robot is
            turning to keep the legs stiff while the ground pulls them apart.
    """

    crouch_height: float
    crouch_velocity: float
    gain_scale: float
    max_crouch_height: float
    max_crouch_velocity: float
    max_joint_velocity: float
    turning_gain_scale: float

    def __init__(
        self,
        config: Dict[str, Any],
        gain_scale: float,
        max_crouch_height: float,
        max_crouch_velocity: float,
        max_joint_velocity: float,
        turning_gain_scale: float,
        wheel_distance: float,
    ):
        """
        Create controller.

        Args:
            config: Global configuration dictionary.
            gain_scale: PD gain scale for hip and knee joints.
            max_crouch_height: Maximum distance along the vertical axis that
                the robot goes down while crouching, in meters.
            max_crouch_velocity: Maximum vertical velocity in [m] / [s].
            max_joint_velocity: Maximum joint angular velocity in [rad] / [s].
            turning_gain_scale: Additional gain scale added when the robot is
                turning to keep the legs stiff in spite of the ground pulling
                them apart.
            wheel_distance: Lateral distance between the two wheels in meters.
                This controller does not handle the case where the two wheels
                are not in the lateral plane.
        """
        self.average_leg_positions = (np.nan, np.nan)
        self.crouch_height = np.nan
        self.gain_scale = clamp(gain_scale, 0.1, 2.0)
        self.max_crouch_height = max_crouch_height
        self.max_crouch_velocity = max_crouch_velocity
        self.max_joint_velocity = max_joint_velocity
        self.position_right_in_left = np.array([0.0, wheel_distance, 0.0])
        self.turning_gain_scale = turning_gain_scale
        self.wheel_balancer = WheelBalancer()  # type: ignore

    def update_target_crouch(
        self, observation: Dict[str, Any], dt: float
    ) -> None:
        """
        Update target crouch from joystick inputs.

        Args:
            observation: Observation from the spine.
            dt: Duration in seconds until next cycle.
        """
        try:
            axis_value: float = observation["joystick"]["pad_axis"][1]
            velocity = self.max_crouch_velocity * axis_value
        except KeyError:
            velocity = 0.0
        crouch_height = self.crouch_height + velocity * dt
        self.crouch_height = clamp(crouch_height, 0.0, self.max_crouch_height)

    def _initialize_crouch(self, observation: Dict[str, Any]) -> None:
        """
        Initialize crouch and its derivative from first observation.

        Args:
            observation: Observation from the spine.
        """
        q_hip = np.mean(
            [
                observation["servo"][hip]["position"]
                for hip in ["left_hip", "right_hip"]
            ]
        )
        q_knee = np.mean(
            [
                observation["servo"][knee]["position"]
                for knee in ["left_knee", "right_knee"]
            ]
        )
        self.average_leg_positions = (q_hip, q_knee)
        self.leg_positions = {
            f"{side}_{joint}": observation["servo"][f"{side}_{joint}"][
                "position"
            ]
            for side in ["left", "right"]
            for joint in ["hip", "knee"]
        }
        self.crouch_height = forward_kinematics(q_hip, q_knee)

    def cycle(self, observation: Dict[str, Any], dt: float) -> Dict[str, Any]:
        """
        Compute action for a new cycle.

        Args:
            observation: Latest observation.
            dt: Duration in seconds until next cycle.

        Returns:
            Dictionary with the new action and some internal state for logging.
        """
        if np.isnan(self.crouch_height):
            self._initialize_crouch(observation)

        # Wheels
        self.wheel_balancer.cycle(observation, dt)
        w = self.wheel_balancer.get_wheel_velocities(
            self.position_right_in_left
        )
        left_wheel_velocity, right_wheel_velocity = w
        servo_action = {
            "left_wheel": {
                "position": np.nan,
                "velocity": left_wheel_velocity,
            },
            "right_wheel": {
                "position": np.nan,
                "velocity": right_wheel_velocity,
            },
        }

        # Legs (hips and knees)
        self.update_target_crouch(observation, dt)
        q_avg = velocity_limited_inverse_kinematics(
            self.crouch_height, self.average_leg_positions, dt
        )
        self.average_leg_positions = q_avg
        average_hip_position, average_knee_position = q_avg
        leg_targets = {
            "left_hip": -average_hip_position,
            "left_knee": -average_knee_position,
            "right_hip": +average_hip_position,
            "right_knee": +average_knee_position,
        }
        for side in ["left", "right"]:
            for joint in ["hip", "knee"]:
                joint_name = f"{side}_{joint}"
                joint_velocity = velocity_limited_joint_control(
                    leg_targets[joint_name],
                    self.leg_positions[joint_name],
                    dt,
                    max_joint_velocity=self.max_joint_velocity,
                )
                self.leg_positions[joint_name] += dt * joint_velocity
                servo_action[joint_name] = {
                    "position": self.leg_positions[joint_name],
                    "velocity": joint_velocity,
                }

        # Increase leg stiffness while turning
        turning_prob = self.wheel_balancer.turning_probability
        # using the same numbers for both gain scales for now
        kp_scale = self.gain_scale + self.turning_gain_scale * turning_prob
        kd_scale = self.gain_scale + self.turning_gain_scale * turning_prob
        for joint_name in ["left_hip", "left_knee", "right_hip", "right_knee"]:
            servo_action[joint_name]["kp_scale"] = kp_scale
            servo_action[joint_name]["kd_scale"] = kd_scale

        return {
            "servo": servo_action,
            "wheel_balancer": self.wheel_balancer.log(),
        }
