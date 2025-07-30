#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron
# Copyright 2023 Inria

"""Control wheels to track ground and yaw velocity targets."""

import gin
import numpy as np

from upkie.control.mpc_balancer import MPCBalancer
from upkie.utils.clamp import clamp, clamp_abs
from upkie.utils.filters import abs_bounded_derivative_filter

from .remote_control import RemoteControl


@gin.configurable
class WheelController:
    """Base class for wheel balancers.

    Attributes:
        sagittal_balancer: Internal controller for sagittal balance.
        target_ground_velocity: Target ground sagittal velocity in [m] / [s].
        target_yaw_velocity: Target yaw velocity in [rad] / [s].
        turning_deadband: Joystick axis value between 0.0 and 1.0 below which
            legs stiffen but the turning motion doesn't start.
        turning_probability: Probability that the user wants to turn based on
            the joystick axis value.
        turning_decision_time: Minimum duration in [s] for the turning
            probability to switch from zero to one and converesly.
        wheel_radius: Wheel radius in [m].
    """

    left_wheeled: bool
    sagittal_balancer: MPCBalancer
    target_ground_velocity: float
    target_yaw_velocity: float
    turning_deadband: float
    turning_decision_time: float
    turning_probability: float
    wheel_radius: float

    def __init__(
        self,
        fall_pitch: float,
        left_wheeled: bool,
        max_ground_accel: float,
        max_ground_velocity: float,
        turning_deadband: float,
        turning_decision_time: float,
        wheel_radius: float,
    ):
        """Initialize balancer.

        Args:
            fall_pitch: Fall pitch threshold, in radians.
            left_wheeled: Set to True (default) if the robot is left wheeled,
                that is, a positive turn of the left wheel results in forward
                motion. Set to False for a right-wheeled variant.
            turning_deadband: Joystick axis value between 0.0 and 1.0 below
                which legs stiffen but the turning motion doesn't start.
            turning_decision_time: Minimum duration in [s] for the turning
                probability to switch from zero to one and converesly.
            wheel_radius: Wheel radius in [m].
            max_ground_accel: Maximum commanded ground acceleration.
            max_ground_velocity: Maximum commanded ground velocity.
        """
        assert 0.0 <= turning_deadband <= 1.0
        sagittal_balancer = MPCBalancer(
            fall_pitch=fall_pitch,
            max_ground_accel=max_ground_accel,
            max_ground_velocity=max_ground_velocity,
        )
        self.left_wheeled = left_wheeled
        self.remote_control = RemoteControl()
        self.sagittal_balancer = sagittal_balancer
        self.target_ground_velocity = 0.0
        self.target_yaw_velocity = 0.0
        self.turning_deadband = turning_deadband
        self.turning_decision_time = turning_decision_time
        self.turning_probability = 0.0
        self.wheel_radius = wheel_radius

    def log(self) -> dict:
        """Log internal state to a dictionary.

        Returns:
            Log data as a dictionary.
        """
        return {
            "commanded_velocity": self.sagittal_balancer.commanded_velocity,
            "target_ground_velocity": self.target_ground_velocity,
            "target_yaw_velocity": self.target_yaw_velocity,
        }

    def cycle(self, observation: dict, dt: float) -> dict:
        """Compute a new ground velocity.

        Args:
            observation: Latest observation.
            dt: Time in [s] until next cycle.

        Returns:
            New ground velocity, in [m] / [s].
        """
        self.update_target_ground_velocity(observation, dt)
        self.update_target_yaw_velocity(observation, dt)

        ground_velocity = self.sagittal_balancer.compute_ground_velocity(
            self.target_ground_velocity, observation, dt
        )

        # Sagittal translation
        wheel_velocity = ground_velocity / self.wheel_radius
        left_sign: float = 1.0 if self.left_wheeled else -1.0
        right_sign = -left_sign
        left_wheel_velocity = left_sign * wheel_velocity
        right_wheel_velocity = right_sign * wheel_velocity

        # Yaw rotation
        delta = observation["height_controller"]["position_right_in_left"]
        contact_radius = 0.5 * np.linalg.norm(delta)
        yaw_to_wheel = left_sign * contact_radius / self.wheel_radius
        left_wheel_velocity += yaw_to_wheel * self.target_yaw_velocity
        right_wheel_velocity += yaw_to_wheel * self.target_yaw_velocity

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
        return {"servo": servo_action}

    def update_target_ground_velocity(
        self, observation: dict, dt: float
    ) -> None:
        """Update target ground velocity from joystick input.

        Args:
            observation: Latest observation.
            dt: Time in [s] until next cycle.

        Note:
            The target ground velocity is commanded by both the left axis and
            right trigger of the joystick. When the right trigger is unpressed,
            the commanded velocity is set from the left axis, interpolating
            from 0 to 50% of its maximum configured value. Pressing the right
            trigger increases it further up to 100% of the configured value.
        """
        try:
            axis_value = observation["joystick"]["left_axis"][1]
            max_velocity = self.remote_control.max_linear_velocity
            unfiltered_velocity = -max_velocity * axis_value
        except KeyError:
            unfiltered_velocity = 0.0
        unclipped_ground_velocity = abs_bounded_derivative_filter(
            prev_output=self.target_ground_velocity,
            new_input=unfiltered_velocity,
            dt=dt,
            max_derivative=self.remote_control.max_linear_accel,
        )
        self.target_ground_velocity = clamp_abs(
            unclipped_ground_velocity,
            self.remote_control.max_linear_velocity,
        )

    def update_target_yaw_velocity(self, observation: dict, dt: float) -> None:
        """Update target yaw velocity from joystick input.

        Args:
            observation: Latest observation.
            dt: Time in [s] until next cycle.
        """
        try:
            joystick_value = observation["joystick"]["right_axis"][0]
        except KeyError:
            joystick_value = 0.0
        joystick_abs = abs(joystick_value)
        joystick_sign = np.sign(joystick_value)

        turning_intent = joystick_abs / self.turning_deadband
        unclipped_probability = abs_bounded_derivative_filter(
            prev_output=self.turning_probability,
            new_input=turning_intent,  # might be > 1.0
            dt=dt,
            max_derivative=1.0 / self.turning_decision_time,
        )
        self.turning_probability = clamp(unclipped_probability, 0.0, 1.0)

        velocity_ratio = (joystick_abs - self.turning_deadband) / (
            1.0 - self.turning_deadband
        )
        velocity_ratio = max(0.0, velocity_ratio)
        max_yaw_velocity = self.remote_control.max_yaw_velocity
        velocity = max_yaw_velocity * joystick_sign * velocity_ratio
        turn_hasnt_started = abs(self.target_yaw_velocity) < 0.01
        turn_not_sure_yet = self.turning_probability < 0.99
        if turn_hasnt_started and turn_not_sure_yet:
            velocity = 0.0
        unclipped_yaw_velocity = abs_bounded_derivative_filter(
            prev_output=self.target_yaw_velocity,
            new_input=velocity,
            dt=dt,
            max_derivative=self.remote_control.max_yaw_accel,
        )
        self.target_yaw_velocity = clamp_abs(
            unclipped_yaw_velocity,
            self.remote_control.max_yaw_velocity,
        )
        if abs(self.target_yaw_velocity) > 0.01:  # still turning
            self.turning_probability = 1.0
