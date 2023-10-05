#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2022 Stéphane Caron
# SPDX-License-Identifier: Apache-2.0

"""
Keep Upkie up! Using its wheels.
"""

from typing import Any, Dict, Tuple

import gin
import numpy as np

from upkie.observers.base_pitch import compute_base_pitch_from_imu
from upkie.utils.clamp import clamp, clamp_abs
from upkie.utils.exceptions import FallDetected
from upkie.utils.filters import abs_bounded_derivative_filter, low_pass_filter


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
    gains: Gains
    ground_velocity: float
    integral_error_velocity: float
    max_ground_velocity: float
    max_integral_error_velocity: float
    max_target_accel: float
    max_target_distance: float
    max_target_velocity: float
    pitch: float
    target_ground_position: float
    target_ground_velocity: float
    target_yaw_position: float
    target_yaw_velocity: float
    turning_deadband: float
    turning_decision_time: float
    turning_probability: float
    wheel_radius: float

    def __init__(
        self,
        air_return_period: float,
        fall_pitch: float,
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
    ):
        """
        Initialize balancer.

        Args:
            air_return_period: Cutoff period for resetting integrators while
                the robot is in the air, in [s].
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

    def update_target_ground_velocity(
        self, observation: dict, dt: float
    ) -> None:
        """
        Update target ground velocity from joystick input.

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
            trigger_value = observation["joystick"]["right_trigger"]  # -1 to 1
            boost_value = clamp_abs(0.5 * (trigger_value + 1.0), 1.0)  # 0 to 1
            max_velocity = 0.5 * (1.0 + boost_value) * self.max_target_velocity
            unfiltered_velocity = -max_velocity * axis_value
        except KeyError:
            unfiltered_velocity = 0.0
        self.target_ground_velocity = abs_bounded_derivative_filter(
            self.target_ground_velocity,
            unfiltered_velocity,
            dt,
            self.max_target_velocity,
            self.max_target_accel,
        )

    def update_target_yaw_velocity(self, observation: dict, dt: float) -> None:
        """
        Update target yaw velocity from joystick input.

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
        self.turning_probability = abs_bounded_derivative_filter(
            self.turning_probability,
            turning_intent,  # might be > 1.0
            dt,
            max_output=1.0,  # output is <= 1.0
            max_derivative=1.0 / self.turning_decision_time,
        )

        velocity_ratio = (joystick_abs - self.turning_deadband) / (
            1.0 - self.turning_deadband
        )
        velocity_ratio = max(0.0, velocity_ratio)
        velocity = self.max_yaw_velocity * joystick_sign * velocity_ratio
        turn_hasnt_started = abs(self.target_yaw_velocity) < 0.01
        turn_not_sure_yet = self.turning_probability < 0.99
        if turn_hasnt_started and turn_not_sure_yet:
            velocity = 0.0
        self.target_yaw_velocity = abs_bounded_derivative_filter(
            self.target_yaw_velocity,
            velocity,
            dt,
            self.max_yaw_velocity,
            self.max_yaw_accel,
        )
        if abs(self.target_yaw_velocity) > 0.01:  # still turning
            self.turning_probability = 1.0

    def process_joystick_buttons(self, observation: Dict[str, Any]) -> None:
        ground_position = observation["wheel_odometry"]["position"]
        try:
            if observation["joystick"]["cross_button"]:
                # When the user presses the reset button, we assume there is no
                # contact for sure (or we are in a situation where spinning the
                # wheels is dangerous?) and thus perform a hard rather than
                # soft reset of both integrators.
                self.integral_error_velocity = 0.0  # [m] / [s]
                self.target_ground_position = ground_position
        except KeyError:
            pass

    def cycle(self, observation: Dict[str, Any], dt: float) -> None:
        """
        Compute a new ground velocity.

        Args:
            observation: Latest observation.
            dt: Time in [s] until next cycle.
        """
        self.process_joystick_buttons(observation)
        self.update_target_ground_velocity(observation, dt)
        self.update_target_yaw_velocity(observation, dt)

        pitch = compute_base_pitch_from_imu(observation["imu"]["orientation"])
        self.pitch = pitch
        if abs(pitch) > self.fall_pitch:
            self.integral_error_velocity = 0.0  # [m] / [s]
            self.ground_velocity = 0.0  # [m] / [s]
            raise FallDetected(f"Base angle {pitch=:.3} rad denotes a fall")

        ground_position = observation["wheel_odometry"]["position"]
        floor_contact = observation["floor_contact"]["contact"]

        target_pitch: float = 0.0  # [rad]
        error = np.array(
            [
                self.target_ground_position - ground_position,
                target_pitch - pitch,
            ]
        )
        self.error = error

        if not floor_contact:
            self.integral_error_velocity = low_pass_filter(
                self.integral_error_velocity, self.air_return_period, 0.0, dt
            )
            # We don't reset self.target_ground_velocity: either takeoff
            # detection is a false positive and we should resume close to the
            # pre-takeoff state, or the robot is really in the air and the user
            # should stop smashing the joystick like a bittern ;p
            self.target_ground_position = low_pass_filter(
                self.target_ground_position,
                self.air_return_period,
                ground_position,
                dt,
            )
        else:  # floor_contact:
            ki = np.array(
                [
                    self.gains.position_stiffness,
                    self.gains.pitch_stiffness,
                ]
            )
            self.integral_error_velocity += ki.dot(error) * dt
            self.integral_error_velocity = clamp_abs(
                self.integral_error_velocity, self.max_integral_error_velocity
            )
            self.target_ground_position += self.target_ground_velocity * dt
            self.target_ground_position = clamp(
                self.target_ground_position,
                ground_position - self.max_target_distance,
                ground_position + self.max_target_distance,
            )

        kp = np.array(
            [
                self.gains.position_damping,
                self.gains.pitch_damping,
            ]
        )

        # Non-minimum phase trick: as per control theory's book, the proper
        # feedforward velocity should be ``+self.target_ground_velocity``.
        # However, it is with resolute purpose that it sends
        # ``-self.target_ground_velocity`` instead!
        #
        # Try both on the robot, you will see the difference :)
        #
        # This hack is not purely out of "esprit de contradiction". Changing
        # velocity is a non-minimum phase behavior (to accelerate forward, the
        # ZMP of the LIPM needs to move backward at first, then forward), and
        # our feedback can't realize that (it only takes care of balancing
        # around a stationary velocity).
        #
        # What's left? Our integrator! If we send the opposite of the target
        # velocity (or only a fraction of it, although 100% seems to do a good
        # job), Upkie will immediately start executing the desired non-minimum
        # phase behavior. The error will then grow and the integrator catch up
        # so that ``upkie_trick_velocity - self.integral_error_velocity``
        # converges to its proper steady state value (the same value ``0 -
        # self.integral_error_velocity`` would have converged to if we had no
        # feedforward).
        #
        # Unconvinced? Try it on the robot. You will feel Upkie's trick ;)
        #
        upkie_trick_velocity = -self.target_ground_velocity

        self.ground_velocity = (
            upkie_trick_velocity - kp.dot(error) - self.integral_error_velocity
        )
        self.ground_velocity = clamp_abs(
            self.ground_velocity, self.max_ground_velocity
        )

    def get_wheel_velocities(
        self,
        position_right_in_left: np.ndarray,
    ) -> Tuple[float, float]:
        """
        Get left and right wheel velocities.

        Args:
            position_right_in_left: Translation from the left contact frame to
                the right contact frame, expressed in the left contact frame.
                Equivalently, linear coordinates of the pose of the right
                contact frame with respect to the left contact frame.

        Note:
            For now we assume that the two wheels are parallel to the ground,
            so that the rotation from one frame to the other is the identity.

        Returns:
            left_wheel_velocity: Left wheel velocity in [rad] / [s].
            right_wheel_velocity: Right wheel velocity in [rad] / [s].
        """
        # Sagittal translation
        left_wheel_velocity: float = +self.ground_velocity / self.wheel_radius
        right_wheel_velocity: float = -self.ground_velocity / self.wheel_radius

        # Yaw rotation
        contact_radius = 0.5 * np.linalg.norm(position_right_in_left)
        yaw_to_wheel = contact_radius / self.wheel_radius
        left_wheel_velocity += yaw_to_wheel * self.target_yaw_velocity
        right_wheel_velocity += yaw_to_wheel * self.target_yaw_velocity

        return left_wheel_velocity, right_wheel_velocity

    def log(self) -> dict:
        """
        Log internal state to a dictionary.

        Returns:
            Log data as a dictionary.
        """
        return {
            "error": self.error,
            "gains": self.gains.__dict__,
            "ground_velocity": self.ground_velocity,
            "integral_error_velocity": self.integral_error_velocity,
            "pitch": self.pitch,
            "target_ground_position": self.target_ground_position,
            "target_ground_velocity": self.target_ground_velocity,
            "target_yaw_velocity": self.target_yaw_velocity,
        }
