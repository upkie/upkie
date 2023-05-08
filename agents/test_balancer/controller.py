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
    return q


@gin.configurable
class Controller:

    """Balance Upkie using its wheels.

    Attributes:
        gain_scale: PD gain scale for hip and knee joints.
        position_right_in_left: Translation from the left contact frame to
            the right contact frame, expressed in the left contact frame.
        turning_gain_scale: Additional gain scale added when the robot is
            turning to keep the legs stiff while the ground pulls them apart.
    """

    gain_scale: float
    max_joint_velocity: float
    turning_gain_scale: float

    def __init__(
        self,
        config: Dict[str, Any],
        gain_scale: float,
        max_joint_velocity: float,
        turning_gain_scale: float,
        wheel_distance: float,
    ):
        """
        Create controller.

        Args:
            config: Global configuration dictionary.
            gain_scale: PD gain scale for hip and knee joints.
            max_joint_velocity: Maximum joint angular velocity in [rad] / [s].
            turning_gain_scale: Additional gain scale added when the robot is
                turning to keep the legs stiff in spite of the ground pulling
                them apart.
            wheel_distance: Lateral distance between the two wheels in meters.
                This controller does not handle the case where the two wheels
                are not in the lateral plane.
        """
        self.average_leg_positions = (np.nan, np.nan)
        self.gain_scale = clamp(gain_scale, 0.1, 2.0)
        self.max_joint_velocity = max_joint_velocity
        self.position_right_in_left = np.array([0.0, wheel_distance, 0.0])
        self.turning_gain_scale = turning_gain_scale
        self.wheel_balancer = WheelBalancer()  # type: ignore

    def cycle(self, observation: Dict[str, Any], dt: float) -> Dict[str, Any]:
        """
        Compute action for a new cycle.

        Args:
            observation: Latest observation.
            dt: Duration in seconds until next cycle.

        Returns:
            Dictionary with the new action and some internal state for logging.
        """
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
