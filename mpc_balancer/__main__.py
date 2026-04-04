#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

import argparse
import socket
from importlib import import_module
from pathlib import Path
from typing import Optional

import gin
import gymnasium as gym
import numpy as np

import upkie.envs
from upkie.controllers.mpc_balancer import MPCBalancer
from upkie.logging import logger
from upkie.model import Model
from upkie.utils.clamp import clamp, clamp_abs
from upkie.utils.filters import abs_bounded_derivative_filter
from upkie.utils.raspi import configure_agent_process, on_raspi

from .remote_control import RemoteControl


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


def update_target_ground_velocity(
    target_ground_velocity: float,
    remote_control: RemoteControl,
    spine_observation: dict,
    dt: float,
) -> float:
    r"""!
    Update target ground velocity from joystick input.

    \param target_ground_velocity Current target ground velocity in m/s.
    \param remote_control Remote control parameters.
    \param spine_observation Spine observation dictionary.
    \param dt Duration in seconds until the next cycle.
    \return Updated target ground velocity.
    """
    try:
        axis_value = spine_observation["joystick"]["left_axis"][1]
        max_velocity = remote_control.max_linear_velocity
        unfiltered_velocity = -max_velocity * axis_value
    except KeyError:
        unfiltered_velocity = 0.0
    unclipped = abs_bounded_derivative_filter(
        prev_output=target_ground_velocity,
        new_input=unfiltered_velocity,
        dt=dt,
        max_derivative=remote_control.max_linear_accel,
    )
    return clamp_abs(unclipped, remote_control.max_linear_velocity)


def update_target_yaw_velocity(
    target_yaw_velocity: float,
    turning_probability: float,
    turning_deadband: float,
    turning_decision_time: float,
    remote_control: RemoteControl,
    spine_observation: dict,
    dt: float,
) -> tuple:
    r"""!
    Update target yaw velocity from joystick input.

    \param target_yaw_velocity Current target yaw velocity in rad/s.
    \param turning_probability Current turning probability.
    \param turning_deadband Joystick axis value below which legs stiffen but
        the turning motion doesn't start.
    \param turning_decision_time Minimum duration in seconds for the turning
        probability to switch from zero to one and conversely.
    \param remote_control Remote control parameters.
    \param spine_observation Spine observation dictionary.
    \param dt Duration in seconds until the next cycle.
    \return Tuple of updated (target yaw velocity, turning probability).
    """
    try:
        joystick_value = spine_observation["joystick"]["right_axis"][0]
    except KeyError:
        joystick_value = 0.0
    joystick_abs = abs(joystick_value)
    joystick_sign = np.sign(joystick_value)

    turning_intent = joystick_abs / turning_deadband
    unclipped_probability = abs_bounded_derivative_filter(
        prev_output=turning_probability,
        new_input=turning_intent,
        dt=dt,
        max_derivative=1.0 / turning_decision_time,
    )
    turning_probability = clamp(unclipped_probability, 0.0, 1.0)

    velocity_ratio = (joystick_abs - turning_deadband) / (
        1.0 - turning_deadband
    )
    velocity_ratio = max(0.0, velocity_ratio)
    max_yaw_velocity = remote_control.max_yaw_velocity
    velocity = max_yaw_velocity * joystick_sign * velocity_ratio
    turn_hasnt_started = abs(target_yaw_velocity) < 0.01
    turn_not_sure_yet = turning_probability < 0.99
    if turn_hasnt_started and turn_not_sure_yet:
        velocity = 0.0
    unclipped_yaw_velocity = abs_bounded_derivative_filter(
        prev_output=target_yaw_velocity,
        new_input=velocity,
        dt=dt,
        max_derivative=remote_control.max_yaw_accel,
    )
    target_yaw_velocity = clamp_abs(
        unclipped_yaw_velocity,
        remote_control.max_yaw_velocity,
    )
    if abs(target_yaw_velocity) > 0.01:  # still turning
        turning_probability = 1.0

    return target_yaw_velocity, turning_probability


@gin.configurable
def run(
    model: Model,
    gain_scale: float,
    turning_gain_scale: float,
    fall_pitch: float = 1.0,
    leg_length: float = 0.58,
    max_ground_accel: float = 10.0,
    max_ground_velocity: float = 3.0,
    turning_deadband: float = 0.3,
    turning_decision_time: float = 0.2,
    frequency: float = 200.0,
) -> None:
    r"""!
    Run agent using a gyropod environment.

    \param model Robot model.
    \param gain_scale PD gain scale for hip and knee joints.
    \param turning_gain_scale Additional gain scaling applied when turning.
    \param fall_pitch Fall detection pitch angle in radians.
    \param leg_length Leg length in meters.
    \param max_ground_accel Maximum commanded ground acceleration in m/s².
    \param max_ground_velocity Maximum commanded ground velocity in m/s.
    \param turning_deadband Joystick axis value between 0.0 and 1.0 below
        which legs stiffen but the turning motion doesn't start.
    \param turning_decision_time Minimum duration in seconds for the turning
        probability to switch from zero to one and conversely.
    \param frequency Control frequency in Hz.
    """
    upkie.envs.register()

    gain_scale = clamp(gain_scale, 0.1, 2.0)
    remote_control = RemoteControl()
    mpc_balancer = MPCBalancer(
        leg_length=leg_length,
        fall_pitch=fall_pitch,
        max_ground_accel=max_ground_accel,
        max_ground_velocity=max_ground_velocity,
    )

    dt = 1.0 / frequency
    target_ground_velocity = 0.0
    target_yaw_velocity = 0.0
    turning_probability = 0.0

    with gym.make(
        "Upkie-Spine-Gyropod",
        frequency=frequency,
        max_ground_velocity=max_ground_velocity,
        max_yaw_velocity=remote_control.max_yaw_velocity,
    ) as env:
        _, info = env.reset()
        spine_observation = info["spine_observation"]

        while True:
            # Update velocity targets from joystick
            target_ground_velocity = update_target_ground_velocity(
                target_ground_velocity, remote_control, spine_observation, dt
            )
            target_yaw_velocity, turning_probability = (
                update_target_yaw_velocity(
                    target_yaw_velocity,
                    turning_probability,
                    turning_deadband,
                    turning_decision_time,
                    remote_control,
                    spine_observation,
                    dt,
                )
            )

            # MPC computes commanded ground velocity
            ground_velocity = mpc_balancer.compute_ground_velocity(
                target_ground_velocity, spine_observation, dt
            )

            # Update leg gain scaling based on turning probability
            set_leg_gain_scale = env.get_wrapper_attr("set_leg_gain_scale")
            set_leg_gain_scale(
                gain_scale + turning_gain_scale * turning_probability
            )

            action = np.array([ground_velocity, target_yaw_velocity])
            _, _, terminated, truncated, info = env.step(action)
            spine_observation = info["spine_observation"]

            if terminated or truncated:
                _, info = env.reset()
                spine_observation = info["spine_observation"]
                target_ground_velocity = 0.0
                target_yaw_velocity = 0.0
                turning_probability = 0.0


def read_gin_configuration(cli_config: Optional[str]):
    config_dir = Path(__file__).parent / "config"

    default_config = config_dir / "default.gin"
    gin.parse_config_file(default_config)

    hostname = socket.gethostname()
    hostname_config = config_dir / f"{hostname}.gin"
    if hostname_config.exists():
        gin.parse_config_file(hostname_config)

    local_config = Path.home() / ".config" / "upkie" / "mpc_balancer.gin"
    if local_config.exists():
        gin.parse_config_file(local_config)

    if cli_config is not None:
        gin.parse_config_file(config_dir / f"{cli_config}.gin")


if __name__ == "__main__":
    args = parse_command_line_arguments()
    read_gin_configuration(args.config)

    # On Raspberry Pi, configure the process to run on a separate CPU core
    if on_raspi():
        configure_agent_process()

    description = import_module(args.description)
    model = Model(description.URDF_PATH)

    remote_control = RemoteControl()
    logger.info(
        f"Max. remote-control velocity: "
        f"{remote_control.max_linear_velocity} m/s"
    )
    logger.info(f"Wheel radius: {model.wheel_radius} m")

    try:
        run(model)
    except KeyboardInterrupt:
        logger.info("Terminating in response to keyboard interrupt")
