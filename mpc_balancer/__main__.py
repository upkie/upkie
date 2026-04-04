#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

import argparse
from importlib import import_module

import gymnasium as gym
import numpy as np

import upkie.envs
from upkie.controllers.mpc_balancer import MPCBalancer
from upkie.logging import logger
from upkie.model import Model
from upkie.utils.clamp import clamp, clamp_abs
from upkie.utils.filters import abs_bounded_derivative_filter
from upkie.utils.raspi import configure_agent_process, on_raspi


class Controller:
    r"""!
    Base class for wheel balancers.
    """

    ## \var model
    ## Robot model.
    model: Model

    ## \var mpc_balancer
    ## Internal controller for sagittal balance.
    mpc_balancer: MPCBalancer

    ## \var target_ground_velocity
    ## Target ground sagittal velocity in m/s.
    target_ground_velocity: float

    ## \var target_yaw_velocity
    ## Target yaw velocity in rad/s.
    target_yaw_velocity: float

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

    def __init__(
        self,
        model: Model,
        fall_pitch: float = 1.0,
        leg_length: float = 0.58,
        max_ground_accel: float = 10.0,
        max_ground_velocity: float = 3.0,
        turning_deadband: float = 0.3,
        turning_decision_time: float = 0.2,
        max_linear_velocity: float = 1.5,
        max_linear_accel: float = 1.2,
        max_yaw_velocity: float = 1.0,
        max_yaw_accel: float = 10.0,
    ):
        r"""!
        Initialize balancer.

        \param fall_pitch Fall pitch threshold, in radians.
        \param leg_length Leg length in meters.
        \param turning_deadband Joystick axis value between 0.0 and 1.0 below
            which legs stiffen but the turning motion doesn't start.
        \param turning_decision_time Minimum duration in seconds for the
            turning probability to switch from zero to one and conversely.
        \param max_ground_accel Maximum commanded ground acceleration.
        \param max_ground_velocity Maximum commanded ground velocity.
        \param max_linear_accel Maximum acceleration for the ground position
            target, in m/s². Does not affect the commanded ground velocity.
        \param max_linear_velocity Maximum velocity for the ground position
            target, in m/s. Indirectly affects the commanded ground velocity.
        \param max_yaw_accel Maximum yaw angular acceleration in rad/s².
        \param max_yaw_velocity Maximum yaw angular velocity in rad/s.
        """
        assert 0.0 <= turning_deadband <= 1.0
        mpc_balancer = MPCBalancer(
            leg_length=leg_length,
            fall_pitch=fall_pitch,
            max_ground_accel=max_ground_accel,
            max_ground_velocity=max_ground_velocity,
        )
        self.model = model
        self.mpc_balancer = mpc_balancer
        self.target_ground_velocity = 0.0
        self.target_yaw_velocity = 0.0
        self.turning_deadband = turning_deadband
        self.turning_decision_time = turning_decision_time
        self.turning_probability = 0.0
        self.max_linear_accel = max_linear_accel
        self.max_linear_velocity = max_linear_velocity
        self.max_yaw_accel = max_yaw_accel
        self.max_yaw_velocity = max_yaw_velocity

    def log(self) -> dict:
        r"""!
        Log internal state to a dictionary.

        \return Log data as a dictionary.
        """
        return {
            "commanded_velocity": self.mpc_balancer.commanded_velocity,
            "target_ground_velocity": self.target_ground_velocity,
            "target_yaw_velocity": self.target_yaw_velocity,
        }

    def cycle(self, observation: dict, dt: float) -> dict:
        r"""!
        Compute a new ground velocity.

        \param observation Latest observation.
        \param dt Duration in seconds until the next cycle.
        \return New ground velocity, in m/s.
        """
        self.update_target_ground_velocity(observation, dt)
        self.update_target_yaw_velocity(observation, dt)

        ground_velocity = self.mpc_balancer.compute_ground_velocity(
            self.target_ground_velocity, observation, dt
        )

        # Sagittal translation
        wheel_velocity = ground_velocity / self.model.wheel_radius
        left_sign: float = 1.0 if self.model.left_wheeled else -1.0
        right_sign = -left_sign
        left_wheel_velocity = left_sign * wheel_velocity
        right_wheel_velocity = right_sign * wheel_velocity

        # Yaw rotation
        delta = observation["height_controller"]["position_right_in_left"]
        contact_radius = 0.5 * np.linalg.norm(delta)
        yaw_to_wheel = left_sign * contact_radius / self.model.wheel_radius
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

    def run(
        self,
        gain_scale: float = 2.0,
        turning_gain_scale: float = 2.0,
        frequency: float = 200.0,
    ) -> None:
        r"""!
        Run agent using a gyropod environment.

        \param gain_scale PD gain scale for hip and knee joints.
        \param turning_gain_scale Additional gain scaling applied when turning.
        \param frequency Control frequency in Hz.
        """
        upkie.envs.register()

        gain_scale = clamp(gain_scale, 0.1, 2.0)
        dt = 1.0 / frequency

        with gym.make(
            "Upkie-Spine-Gyropod",
            frequency=frequency,
            max_ground_velocity=self.max_linear_velocity,
            max_yaw_velocity=self.max_yaw_velocity,
        ) as env:
            _, info = env.reset()
            spine_observation = info["spine_observation"]

            while True:
                # Update velocity targets from joystick
                self.update_target_ground_velocity(spine_observation, dt)
                self.update_target_yaw_velocity(spine_observation, dt)

                # MPC computes commanded ground velocity
                ground_velocity = self.mpc_balancer.compute_ground_velocity(
                    self.target_ground_velocity, spine_observation, dt
                )

                # Update leg gain scaling based on turning probability
                set_leg_gain_scale = env.get_wrapper_attr("set_leg_gain_scale")
                set_leg_gain_scale(
                    gain_scale + turning_gain_scale * self.turning_probability
                )

                action = np.array([ground_velocity, self.target_yaw_velocity])
                _, _, terminated, truncated, info = env.step(action)
                spine_observation = info["spine_observation"]

                if terminated or truncated:
                    _, info = env.reset()
                    spine_observation = info["spine_observation"]
                    self.target_ground_velocity = 0.0
                    self.target_yaw_velocity = 0.0
                    self.turning_probability = 0.0


def parse_command_line_arguments() -> argparse.Namespace:
    r"""!
    Parse command line arguments.

    \return Command-line arguments.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "-c",
        "--config",
        metavar="config",
        help="Additional agent configuration to apply",
        type=str,
    )
    parser.add_argument(
        "--description",
        help="Robot description to read (default: upkie_description)",
        choices=["upkie_description", "cookie_description"],
        default="upkie_description",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_command_line_arguments()

    # On Raspberry Pi, configure the process to run on a separate CPU core
    if on_raspi():
        configure_agent_process()

    description = import_module(args.description)
    model = Model(description.URDF_PATH)

    logger.info(f"Wheel radius: {model.wheel_radius} m")

    controller = Controller(model)
    logger.info(
        f"Max. commanded linear velocity: "
        f"{controller.max_linear_velocity} m/s"
    )
    try:
        controller.run()
    except KeyboardInterrupt:
        logger.info("Terminating in response to keyboard interrupt")
