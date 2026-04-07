#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

"""Joystick controller."""

from typing import Literal

import numpy as np

from upkie.utils.clamp import clamp, clamp_abs
from upkie.utils.filters import abs_bounded_derivative_filter


class JoystickController:
    r"""!
    Controller that maps joystick inputs to linear and angular velocities.
    """

    ## \var max_linear_accel
    ## Maximum linear acceleration in m/s².
    max_linear_accel: float

    ## \var max_linear_velocity
    ## Maximum linear velocity in m/s.
    max_linear_velocity: float

    ## \var max_yaw_accel
    ## Maximum yaw angular acceleration in rad/s².
    max_yaw_accel: float

    ## \var max_yaw_velocity
    ## Maximum yaw angular velocity in rad/s.
    max_yaw_velocity: float

    ## \var turning_deadband
    ## Joystick axis value between 0.0 and 1.0 below which turning probability
    ## goes up but turning doesn't start yet.
    turning_deadband: float

    ## \var turning_decision_time
    ## Minimum duration in seconds for the turning probability to switch from
    ## zero to one and conversely.
    turning_decision_time: float

    def __init__(
        self,
        joystick_axis: Literal["left_axis", "right_axis"] = "left_axis",
        max_linear_accel: float = 1.2,
        max_linear_velocity: float = 1.5,
        max_yaw_accel: float = 10.0,
        max_yaw_velocity: float = 1.0,
        turning_deadband: float = 0.3,
        turning_decision_time: float = 0.2,
    ):
        r"""!
        Initialize joystick controller.

        \param joystick_axis Joystick axis to read inputs from. Up-down stick
            motions control the linear velocity, while left-right motions
            control the angular velocity.
        \param max_linear_accel Maximum linear acceleration in m/s².
        \param max_linear_velocity Maximum linear velocity in m/s.
        \param max_yaw_accel Maximum yaw angular acceleration in rad/s².
        \param max_yaw_velocity Maximum yaw angular velocity in rad/s.
        \param turning_deadband Joystick axis value between 0.0 and 1.0 below
            which turning probability goes up but turning doesn't start yet.
        \param turning_decision_time Minimum duration in seconds for the
            turning probability to switch from zero to one and conversely.
        """
        assert 0.0 <= turning_deadband <= 1.0
        assert joystick_axis in ["left_axis", "right_axis"]
        self.__axis_name = joystick_axis
        self.__linear_velocity = 0.0
        self.__turning_probability = 0.0
        self.__yaw_velocity = 0.0
        self.max_linear_accel = max_linear_accel
        self.max_linear_velocity = max_linear_velocity
        self.max_yaw_accel = max_yaw_accel
        self.max_yaw_velocity = max_yaw_velocity
        self.turning_deadband = turning_deadband
        self.turning_decision_time = turning_decision_time

    @property
    def turning_probability(self) -> float:
        r"""!
        Probability the user wants to turn based on the joystick axis value.

        This is an extra output that not returned by the `step` function but
        available if needed.
        """
        return self.__turning_probability

    def reset(self) -> None:
        r"""!
        Reset the internal state of the controller.
        """
        self.__linear_velocity = 0.0
        self.__turning_probability = 0.0
        self.__yaw_velocity = 0.0

    def update_linear_velocity(self, observation: dict, dt: float) -> None:
        r"""!
        Update linear velocity from joystick input.

        \param observation Latest observation.
        \param dt Duration in seconds until the next cycle.
        """
        try:
            axis_value = observation["joystick"][self.__axis_name][1]
            max_velocity = self.max_linear_velocity
            unfiltered_velocity = -max_velocity * axis_value
        except KeyError:
            unfiltered_velocity = 0.0
        unclipped_linear_velocity = abs_bounded_derivative_filter(
            prev_output=self.__linear_velocity,
            new_input=unfiltered_velocity,
            dt=dt,
            max_derivative=self.max_linear_accel,
        )
        self.__linear_velocity = clamp_abs(
            unclipped_linear_velocity,
            self.max_linear_velocity,
        )

    def update_yaw_velocity(self, observation: dict, dt: float) -> None:
        r"""!
        Update yaw angular velocity from joystick input.

        \param observation Latest observation.
        \param dt Duration in seconds until the next cycle.
        """
        try:
            joystick_value = observation["joystick"][self.__axis_name][0]
        except KeyError:
            joystick_value = 0.0
        joystick_abs = abs(joystick_value)
        joystick_sign = np.sign(joystick_value)

        turning_intent = joystick_abs / self.turning_deadband
        unclipped_probability = abs_bounded_derivative_filter(
            prev_output=self.__turning_probability,
            new_input=turning_intent,  # might be > 1.0
            dt=dt,
            max_derivative=1.0 / self.turning_decision_time,
        )
        self.__turning_probability = clamp(unclipped_probability, 0.0, 1.0)

        velocity_ratio = (joystick_abs - self.turning_deadband) / (
            1.0 - self.turning_deadband
        )
        velocity_ratio = max(0.0, velocity_ratio)
        max_yaw_velocity = self.max_yaw_velocity
        velocity = max_yaw_velocity * joystick_sign * velocity_ratio

        turn_hasnt_started = abs(self.__yaw_velocity) < 0.01
        turn_not_sure_yet = self.__turning_probability < 0.99
        if turn_hasnt_started and turn_not_sure_yet:
            velocity = 0.0

        unclipped_yaw_velocity = abs_bounded_derivative_filter(
            prev_output=self.__yaw_velocity,
            new_input=velocity,
            dt=dt,
            max_derivative=self.max_yaw_accel,
        )
        self.__yaw_velocity = clamp_abs(
            unclipped_yaw_velocity,
            self.max_yaw_velocity,
        )
        if abs(self.__yaw_velocity) > 0.01:  # still turning
            self.__turning_probability = 1.0

    def step(self, observation: dict, dt: float) -> tuple[float, float]:
        r"""!
        Get new linear-angular velocities based on joystick input.

        \param observation Observation dictionary containing joystick inputs.
        \param dt Duration in seconds until the next control cycle.
        \return Tuple of (linear_velocity, yaw_velocity).
        """
        self.update_linear_velocity(observation, dt)
        self.update_yaw_velocity(observation, dt)
        return (self.__linear_velocity, self.__yaw_velocity)
