#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron
# Copyright 2023 Inria

import argparse
import socket
import traceback
from pathlib import Path
from typing import Optional

import gin
from loop_rate_limiters import RateLimiter

import upkie.config
from upkie.spine import SpineInterface
from upkie.utils.raspi import configure_agent_process, on_raspi
from upkie.utils.spdlog import logging

from .whole_body_controller import WholeBodyController


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


def run(
    spine: SpineInterface,
    spine_config: dict,
    controller: WholeBodyController,
    frequency: float = 200.0,
) -> None:
    r"""!
    Read observations and send actions to the spine.

    \param spine Interface to the spine.
    \param spine_config Spine configuration dictionary.
    \param controller Whole-body controller.
    \param frequency Control frequency in Hz.
    """
    dt = 1.0 / frequency
    rate = RateLimiter(frequency, "controller")
    observation = spine.start(spine_config)
    while True:
        action = controller.cycle(observation, dt)
        observation = spine.set_action(action)
        rate.sleep()


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

    spine = SpineInterface(retries=10)
    controller = WholeBodyController(visualize=args.visualize)
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
    wheel_controller = controller.wheel_controller
    wheel_radius = wheel_controller.wheel_radius
    wheel_odometry = spine_config["wheel_odometry"]
    left_sign: float = 1.0 if wheel_controller.left_wheeled else -1.0
    right_sign = -left_sign
    wheel_odometry["signed_radius"]["left_wheel"] = left_sign * wheel_radius
    wheel_odometry["signed_radius"]["right_wheel"] = right_sign * wheel_radius

    max_rc_vel = wheel_controller.remote_control.max_linear_velocity
    max_ground_vel = wheel_controller.sagittal_balancer.max_ground_velocity
    logging.info(f"Knees bend {controller.height_controller.knee_side}")
    logging.info(f"Max. remote-control velocity: {max_rc_vel} m/s")
    logging.info(f"Max. commanded velocity: {max_ground_vel} m/s")
    logging.info(f"Wheel radius: {wheel_radius} m")
    logging.info(f"Additional spine config:\n\n{spine_config}\n\n")

    try:
        run(spine, spine_config, controller)
    except KeyboardInterrupt:
        logging.info("Caught a keyboard interrupt")
    except Exception:
        logging.error("Controller raised an exception")
        print("")
        traceback.print_exc()
        print("")

    logging.info("Stopping the spine...")
    try:
        spine.stop()
    except Exception:
        logging.error("Error while stopping the spine!")
        print("")
        traceback.print_exc()
        print("")
