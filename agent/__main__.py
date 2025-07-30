#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron
# Copyright 2023 Inria

import argparse
import socket
from pathlib import Path
from typing import Optional

import gin
import gymnasium as gym

import upkie.config
import upkie.envs
from upkie.utils.clamp import clamp
from upkie.utils.raspi import configure_agent_process, on_raspi
from upkie.utils.spdlog import logging

from .height_controller import HeightController
from .wheel_controller import WheelController


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
        "--visualize",
        help="Publish robot visualization to MeshCat for debugging",
        default=False,
        action="store_true",
    )
    return parser.parse_args()


@gin.configurable
def run(
    spine_config: dict,
    gain_scale: float,
    turning_gain_scale: float,
    visualize: bool,
    frequency: float = 200.0,
) -> None:
    r"""!
    Run agent using the Upkie-Servos-Spine environment.

    \param spine_config Spine configuration dictionary.
    \param gain_scale PD gain scale for hip and knee joints.
    \param turning_gain_scale Additional gain scaling applied when turning.
    \param visualize If true, open a MeshCat visualizer on the side.
    \param frequency Control frequency in Hz.
    """
    upkie.envs.register()

    gain_scale = clamp(gain_scale, 0.1, 2.0)
    height_controller = HeightController(visualize=visualize)
    wheel_controller = WheelController()

    dt = 1.0 / frequency

    with gym.make(
        "Upkie-Servos-Spine", frequency=frequency, spine_config=spine_config
    ) as env:
        _, info = env.reset()
        spine_observation = info["spine_observation"]

        while True:
            leg_action = height_controller.cycle(spine_observation, dt)
            wheel_action = wheel_controller.cycle(spine_observation, dt)
            action = {
                "left_hip": leg_action["servo"]["left_hip"],
                "left_knee": leg_action["servo"]["left_knee"],
                "left_wheel": wheel_action["servo"]["left_wheel"],
                "right_hip": leg_action["servo"]["right_hip"],
                "right_knee": leg_action["servo"]["right_knee"],
                "right_wheel": wheel_action["servo"]["right_wheel"],
            }
            turning_prob = wheel_controller.turning_probability
            kp_scale = gain_scale + turning_gain_scale * turning_prob
            kd_scale = gain_scale + turning_gain_scale * turning_prob
            for joint in env.unwrapped.model.upper_leg_joints:
                action[joint.name]["kp_scale"] = kp_scale
                action[joint.name]["kd_scale"] = kd_scale

            _, _, terminated, truncated, info = env.step(action)
            spine_observation = info["spine_observation"]

            if terminated or truncated:
                _, info = env.reset()
                spine_observation = info["spine_observation"]


def read_gin_configuration(cli_config: Optional[str]):
    config_dir = Path(__file__).parent / "config"

    default_config = config_dir / "default.gin"
    gin.parse_config_file(default_config)

    hostname = socket.gethostname()
    hostname_config = config_dir / f"{hostname}.gin"
    if hostname_config.exists():
        gin.parse_config_file(hostname_config)

    local_config = Path.home() / ".config" / "upkie" / "pink_balancer.gin"
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

    spine_config = upkie.config.SPINE_CONFIG.copy()
    # spine_config["base_orientation"] = {
    #     "rotation_base_to_imu": np.array(
    #         [-1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0],
    #         dtype=float,
    #     )
    # }
    spine_config["bullet"]["reset"]["joint_configuration"] = [
        0.1,
        0.2,
        0.0,
        0.1,
        0.2,
        0.0,
    ]

    # Create temporary controllers to access configuration
    temp_wheel_controller = WheelController()
    wheel_radius = temp_wheel_controller.wheel_radius
    wheel_odometry = spine_config["wheel_odometry"]
    left_sign: float = 1.0 if temp_wheel_controller.left_wheeled else -1.0
    right_sign = -left_sign
    wheel_odometry["signed_radius"]["left_wheel"] = left_sign * wheel_radius
    wheel_odometry["signed_radius"]["right_wheel"] = right_sign * wheel_radius

    max_rc_vel = temp_wheel_controller.remote_control.max_linear_velocity
    max_ground_vel = (
        temp_wheel_controller.sagittal_balancer.max_ground_velocity
    )
    temp_height_controller = HeightController(visualize=False)
    logging.info(f"Knees bend {temp_height_controller.knee_side}")
    logging.info(f"Max. remote-control velocity: {max_rc_vel} m/s")
    logging.info(f"Max. commanded velocity: {max_ground_vel} m/s")
    logging.info(f"Wheel radius: {wheel_radius} m")
    logging.info(f"Additional spine config:\n\n{spine_config}\n\n")

    try:
        run(spine_config, visualize=args.visualize)
    except KeyboardInterrupt:
        logging.info("Terminating in response to keyboard interrupt")
