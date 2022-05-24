#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2022 Stéphane Caron

"""
Keep Upkie up! Using its wheels.
"""

from typing import Any, Dict, Tuple

import gin
import numpy as np
import pinocchio as pin
from utils.clamp import clamp, clamp_abs
from utils.filters import abs_bounded_derivative_filter, low_pass_filter
from utils.imu import compute_base_pitch_from_imu


@gin.configurable
class WheelBalancer:

    """
    Balancing by proportional-derivative feedback of the body pitch error to
    wheel accelerations:

        body pitch error --(PD)--> wheel accelerations

    Attributes:
        air_return_period: Cutoff period for resetting integrators while the
            robot is in the air, in [s].
        error: Two-dimensional vector of ground position and base pitch errors.
        flexion_correction_gain: Gain applied to the hackish flexion error to
            compensate undesired leg flexion, particularly while turning.
        gains: velocity controller gains.
        ground_velocity: Sagittal velocity in [m] / [s].
        integral_error_velocity: Integral term contributing to the sagittal
            velocity, in [m] / [s].
        max_ground_velocity: Maximum commanded ground velocity no matter what,
            in [m] / [s].
        max_integral_error_velocity: Maximum integral error velocity, in [m] /
            [s].
        max_target_accel: Maximum acceleration for the ground target, in
            [m] / [s]². Does not affect the commanded ground velocity.
        max_target_distance: Maximum distance from the current ground position
            to the target, in [m].
        max_target_velocity: Maximum velocity for the ground target, in
            [m] / [s]. Indirectly affects the commanded ground velocity.
        pitch: Current IMU pitch angle in [rad].
        target_ground_position: Target ground sagittal position in [m].
        target_ground_velocity: Target ground sagittal velocity in [m] / [s].
        target_yaw_position: Target yaw position in [rad].
        target_yaw_velocity: Target yaw velocity in [rad] / [s].
        turning_deadband: Joystick axis value between 0.0 and 1.0 below which
            legs stiffen but the turning motion doesn't start.
        turning_probability: Probability that the user wants to turn based on
            the joystick axis value.
        turning_decision_time: Minimum duration in [s] for the turning
            probability to switch from zero to one and converesly.
        wheel_radius: Wheel radius in [m].
    """

    @gin.configurable
    class Gains:
        pitch_damping: float
        pitch_stiffness: float
        position_damping: float
        position_stiffness: float

        def __init__(
            self,
            pitch_damping: float,
            pitch_stiffness: float,
            position_damping: float,
            position_stiffness: float,
        ):
            self.pitch_damping = pitch_damping
            self.pitch_stiffness = pitch_stiffness
            self.position_damping = position_damping
            self.position_stiffness = position_stiffness

        def set(
            self,
            pitch_damping: float,
            pitch_stiffness: float,
            position_damping: float,
            position_stiffness: float,
        ) -> None:
            """
            Set gains in one function call.

            Args:
                pitch_damping: Pitch error (normalized) damping gain.
                    Corresponds to the proportional term of the velocity PI
                    controller, equivalent to the derivative term of the
                    acceleration PD controller.
                pitch_stiffness: Pitch error (normalized) stiffness gain.
                    Corresponds to the integral term of the velocity PI
                    controller, equivalent to the proportional term of the
                    acceleration PD controller.
                position_damping: Position error (normalized) damping gain.
                    Corresponds to the proportional term of the velocity PI
                    controller, equivalent to the derivative term of the
                    acceleration PD controller.
                position_stiffness: Position error (normalized) stiffness gain.
                    Corresponds to the integral term of the velocity PI
                    controller, equivalent to the proportional term of the
                    acceleration PD controller.
            """
            self.pitch_damping = pitch_damping
            self.pitch_stiffness = pitch_stiffness
            self.position_damping = position_damping
            self.position_stiffness = position_stiffness

        def __repr__(self):
            return (
                "WheelBalancer.Gains("
                f"pitch_damping={self.pitch_damping}, "
                f"pitch_stiffness={self.pitch_stiffness}, "
                f"position_damping={self.position_damping}, "
                f"position_stiffness={self.position_stiffness}, "
            )

    air_return_period: float
    error: np.ndarray
    flexion_correction_gain: float
    gains: Gains
    ground_velocity: float
    integral_error_velocity: float
    max_ground_velocity: float
    max_integral_error_velocity: float
    max_target_velocity: float
    max_target_distance: float
    max_target_accel: float
    pitch: float
    target_ground_position: float
    target_ground_velocity: float
    target_yaw_position: float
    target_yaw_velocity: float
    turning_deadband: float
    turning_probability: float
    turning_decision_time: float
    wheel_radius: float

    def __init__(
        self,
        air_return_period: float,
        flexion_correction_gain: float,
        max_ground_velocity: float,
        max_integral_error_velocity: float,
        max_target_accel: float,
        max_target_distance: float,
        max_target_velocity: float,
        max_yaw_accel: float,
        max_yaw_velocity: float,
        turning_deadband: float,
        turning_decision_time: float,
        wheel_radius: float,
        fall_pitch: float = 1.0,
    ):
        """
        Initialize balancer.

        Args:
            air_return_period: Cutoff period for resetting integrators while
                the robot is in the air, in [s].
            flexion_correction_gain: Gain applied to the hackish flexion error
                to compensate undesired leg flexion, particularly while
                turning.
            max_ground_velocity: Maximum commanded ground velocity no matter
                what, in [m] / [s].
            max_integral_error_velocity: Maximum integral error velocity, in
                [m] / [s].
            max_target_accel: Maximum acceleration for the ground
                target, in [m] / [s]². This bound does not affect the commanded
                ground velocity.
            max_target_distance: Maximum distance from the current ground
                position to the target, in [m].
            max_target_velocity: Maximum velocity for the ground target,
                in [m] / [s]. This bound indirectly affects the commanded
                ground velocity.
            max_yaw_accel: Maximum yaw angular acceleration in [rad] / [s]².
            max_yaw_velocity: Maximum yaw angular velocity in [rad] / [s].
            turning_deadband: Joystick axis value between 0.0 and 1.0 below
                which legs stiffen but the turning motion doesn't start.
            turning_decision_time: Minimum duration in [s] for the turning
                probability to switch from zero to one and converesly.
            wheel_radius: Wheel radius in [m].
        """
        assert 0.0 <= turning_deadband <= 1.0
        self.air_return_period = air_return_period
        self.error = np.zeros(2)
        self.fall_pitch = fall_pitch
        self.flexion_correction_gain = flexion_correction_gain
        self.gains = WheelBalancer.Gains()  # type: ignore
        self.ground_velocity = 0.0
        self.integral_error_velocity = 0.0
        self.max_ground_velocity = max_ground_velocity
        self.max_integral_error_velocity = max_integral_error_velocity
        self.max_target_accel = max_target_accel
        self.max_target_distance = max_target_distance
        self.max_target_velocity = max_target_velocity
        self.max_yaw_accel = max_yaw_accel
        self.max_yaw_velocity = max_yaw_velocity
        self.pitch = 0.0
        self.target_ground_position = 0.0
        self.target_ground_velocity = 0.0
        self.target_yaw_position = 0.0
        self.target_yaw_velocity = 0.0
        self.turning_deadband = turning_deadband
        self.turning_decision_time = turning_decision_time
        self.turning_probability = 0.0
        self.wheel_radius = wheel_radius
