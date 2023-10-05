#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2022 Stéphane Caron
# SPDX-License-Identifier: Apache-2.0

from typing import Any, Dict

import gin
import numpy as np
from wheel_controller import WheelController

from upkie.utils.clamp import clamp


@gin.configurable
class ServoController:
    """Balance Upkie using its wheels.

    Attributes:
        gain_scale: PD gain scale for hip and knee joints.
        position_right_in_left: Translation from the left contact frame to
            the right contact frame, expressed in the left contact frame.
        turning_gain_scale: Additional gain scale added when the robot is
            turning to keep the legs stiff while the ground pulls them apart.
    """

    gain_scale: float
    turning_gain_scale: float

    def __init__(
        self,
        gain_scale: float,
        turning_gain_scale: float,
        wheel_distance: float,
    ):
        """Create controller.

        Args:
            gain_scale: PD gain scale for hip and knee joints.
            turning_gain_scale: Additional gain scale added when the robot is
                turning to keep the legs stiff in spite of the ground pulling
                them apart.
            wheel_distance: Lateral distance between the two wheels in meters.
                This controller does not handle the case where the two wheels
                are not in the lateral plane.
        """
        self.gain_scale = clamp(gain_scale, 0.1, 2.0)
        self.position_right_in_left = np.array([0.0, wheel_distance, 0.0])
        self.servo_action = None
        self.turning_gain_scale = turning_gain_scale
        self.wheel_balancer = WheelController()  # type: ignore

    def initialize_servo_action(self, observation: Dict[str, Any]) -> None:
        """Initialize default servo action from initial observation.

        Args:
            observation: Initial observation.
        """
        self.servo_action = {
            joint: {
                "position": observation["servo"][joint]["position"],
                "velocity": 0.0,
            }
            for joint in (
                f"{side}_{func}"
                for side in ("left", "right")
                for func in ("hip", "knee")
            )
        }
        self.servo_action.update(
            {
                wheel: {
                    "position": np.nan,
                    "velocity": 0.0,
                }
                for wheel in ("left_wheel", "right_wheel")
            }
        )

    def cycle(self, observation: Dict[str, Any], dt: float) -> Dict[str, Any]:
        """Compute action for a new cycle.

        Args:
            observation: Latest observation.
            dt: Duration in seconds until next cycle.

        Returns:
            Dictionary with the new action and some internal state for logging.
        """
        if self.servo_action is None:
            self.initialize_servo_action(observation)

        # Compute wheel velocities for balancing
        self.wheel_balancer.cycle(observation, dt)
        wheel_velocities = self.wheel_balancer.get_wheel_velocities(
            self.position_right_in_left
        )
        left_wheel_velocity, right_wheel_velocity = wheel_velocities
        self.servo_action["left_wheel"]["velocity"] = left_wheel_velocity
        self.servo_action["right_wheel"]["velocity"] = right_wheel_velocity

        # Increase leg stiffness while turning
        turning_prob = self.wheel_balancer.turning_probability
        kp_scale = self.gain_scale + self.turning_gain_scale * turning_prob
        kd_scale = self.gain_scale + self.turning_gain_scale * turning_prob
        for joint_name in ["left_hip", "left_knee", "right_hip", "right_knee"]:
            self.servo_action[joint_name]["kp_scale"] = kp_scale
            self.servo_action[joint_name]["kd_scale"] = kd_scale

        return {
            "servo": self.servo_action,
            "wheel_balancer": self.wheel_balancer.log(),
        }
