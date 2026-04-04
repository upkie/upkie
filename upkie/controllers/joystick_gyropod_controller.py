#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

"""Joystick controller for gyropod."""

import numpy as np

from upkie.utils.clamp import clamp, clamp_abs
from upkie.utils.filters import abs_bounded_derivative_filter


class JoystickGyropodController:
    r"""!
    Joystick controller for gyropod.
    """

    ## \var max_linear_accel
    ## Maximum acceleration for the ground position target, in m/s². Does
    ## not affect the commanded ground velocity.
    max_linear_accel: float

    ## \var max_linear_velocity
    ## Maximum velocity for the ground position target, in m/s.
    ## Indirectly affects the commanded ground velocity.
    max_linear_velocity: float

    ## \var max_yaw_accel
    ## Maximum yaw angular acceleration in rad/s².
    max_yaw_accel: float

    ## \var max_yaw_velocity
    ## Maximum yaw angular velocity in rad/s.
    max_yaw_velocity: float

    ## \var turning_deadband
    ## Joystick axis value between 0.0 and 1.0 below which legs stiffen but the
    ## turning motion doesn't start.
    turning_deadband: float

    ## \var turning_decision_time
    ## Minimum duration in seconds for the turning probability to switch from
    ## zero to one and conversely.
    turning_decision_time: float

    ## \var turning_probability
    ## Probability the user wants to turn based on the joystick axis value.
    turning_probability: float

    ## \var target_ground_velocity
    ## Target ground sagittal velocity in m/s.
    target_ground_velocity: float

    ## \var target_yaw_velocity
    ## Target yaw velocity in rad/s.
    target_yaw_velocity: float

    def __init__(
        self,
        turning_deadband: float = 0.3,
        turning_decision_time: float = 0.2,
        max_linear_velocity: float = 1.5,
        max_linear_accel: float = 1.2,
        max_yaw_velocity: float = 1.0,
        max_yaw_accel: float = 10.0,
    ):
        r"""!
        Initialize joystick controller.

        \param turning_deadband Joystick axis value between 0.0 and 1.0 below
            which legs stiffen but the turning motion doesn't start.
        \param turning_decision_time Minimum duration in seconds for the
            turning probability to switch from zero to one and conversely.
        \param max_linear_accel Maximum acceleration for the ground position
            target, in m/s². Does not affect the commanded ground velocity.
        \param max_linear_velocity Maximum velocity for the ground position
            target, in m/s. Indirectly affects the commanded ground velocity.
        \param max_yaw_accel Maximum yaw angular acceleration in rad/s².
        \param max_yaw_velocity Maximum yaw angular velocity in rad/s.
        """
        assert 0.0 <= turning_deadband <= 1.0
        self.turning_deadband = turning_deadband
        self.turning_decision_time = turning_decision_time
        self.turning_probability = 0.0
        self.max_linear_accel = max_linear_accel
        self.max_linear_velocity = max_linear_velocity
        self.max_yaw_accel = max_yaw_accel
        self.max_yaw_velocity = max_yaw_velocity
        self.target_ground_velocity = 0.0
        self.target_yaw_velocity = 0.0

    def update_target_ground_velocity(
        self, observation: dict, dt: float
    ) -> None:
        r"""!
        Update target ground velocity from joystick input.

        \param observation Latest observation.
        \param dt Duration in seconds until the next cycle.

        \note The target ground velocity is commanded by both the left axis and
            right trigger of the joystick. When the right trigger is unpressed,
            the commanded velocity is set from the left axis, interpolating
            from 0 to 50% of its maximum configured value. Pressing the right
            trigger increases it further up to 100% of the configured value.
        """
        try:
            axis_value = observation["joystick"]["left_axis"][1]
            max_velocity = self.max_linear_velocity
            unfiltered_velocity = -max_velocity * axis_value
        except KeyError:
            unfiltered_velocity = 0.0
        unclipped_ground_velocity = abs_bounded_derivative_filter(
            prev_output=self.target_ground_velocity,
            new_input=unfiltered_velocity,
            dt=dt,
            max_derivative=self.max_linear_accel,
        )
        self.target_ground_velocity = clamp_abs(
            unclipped_ground_velocity,
            self.max_linear_velocity,
        )

    def update_target_yaw_velocity(self, observation: dict, dt: float) -> None:
        r"""!
        Update target yaw velocity from joystick input.

        \param observation Latest observation.
        \param dt Duration in seconds until the next cycle.
        """
        try:
            joystick_value = observation["joystick"]["left_axis"][0]
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
        max_yaw_velocity = self.max_yaw_velocity
        velocity = max_yaw_velocity * joystick_sign * velocity_ratio
        turn_hasnt_started = abs(self.target_yaw_velocity) < 0.01
        turn_not_sure_yet = self.turning_probability < 0.99
        if turn_hasnt_started and turn_not_sure_yet:
            velocity = 0.0
        unclipped_yaw_velocity = abs_bounded_derivative_filter(
            prev_output=self.target_yaw_velocity,
            new_input=velocity,
            dt=dt,
            max_derivative=self.max_yaw_accel,
        )
        self.target_yaw_velocity = clamp_abs(
            unclipped_yaw_velocity,
            self.max_yaw_velocity,
        )
        if abs(self.target_yaw_velocity) > 0.01:  # still turning
            self.turning_probability = 1.0

    def step(self, observation: dict, dt: float) -> None:
        self.update_target_ground_velocity(observation, dt)
        self.update_target_yaw_velocity(observation, dt)
