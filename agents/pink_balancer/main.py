#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2022 Stéphane Caron
# SPDX-License-Identifier: Apache-2.0

import argparse
import socket
import traceback
from os import path
from typing import Any, Dict

import gin
from loop_rate_limiters import RateLimiter
from vulp.spine import SpineInterface
from whole_body_controller import WholeBodyController

import upkie.config
from upkie.utils.raspi import configure_agent_process, on_raspi
from upkie.utils.spdlog import logging


def parse_command_line_arguments() -> argparse.Namespace:
    """
    Parse command line arguments.

    Returns:
        Command-line arguments.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "-c",
        "--config",
        metavar="config",
        help="Agent configuration to apply",
        type=str,
        required=True,
        choices=["bullet", "pi3hat"],
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
    spine_config: Dict[str, Any],
    controller: WholeBodyController,
    frequency: float = 200.0,
) -> None:
    """
    Read observations and send actions to the spine.

    Args:
        spine: Interface to the spine.
        spine_config: Spine configuration dictionary.
        controller: Whole-body controller.
        frequency: Control frequency in Hz.
    """
    dt = 1.0 / frequency
    rate = RateLimiter(frequency, "controller")

    spine.start(spine_config)
    observation = spine.get_observation()  # pre-reset observation
    while True:
        observation = spine.get_observation()
        action = controller.cycle(observation, dt)
        spine.set_action(action)
        rate.sleep()


def load_gin_configuration(name: str) -> None:
    logging.info(f"Loading configuration '{name}.gin'")
    try:
        gin.parse_config_file(f"{agent_dir}/config/{name}.gin")
    except OSError as e:
        raise FileNotFoundError(f"Configuration '{name}.gin' not found") from e


if __name__ == "__main__":
    args = parse_command_line_arguments()
    agent_dir = path.dirname(__file__)

    # Agent configuration
    load_gin_configuration("common")
    if args.config == "hostname":
        hostname = socket.gethostname().lower()
        logging.info(f"Loading configuration from hostname '{hostname}'")
        load_gin_configuration(hostname)
    elif args.config is not None:
        load_gin_configuration(args.config)

    # On Raspberry Pi, configure the process to run on a separate CPU core
    if on_raspi():
        configure_agent_process()

    spine = SpineInterface()
    controller = WholeBodyController(visualize=args.visualize)
    spine_config = upkie.config.SPINE_CONFIG.copy()
    wheel_radius = controller.wheel_balancer.wheel_radius
    wheel_odometry_config = spine_config["wheel_odometry"]
    wheel_odometry_config["signed_radius"]["left_wheel"] = +wheel_radius
    wheel_odometry_config["signed_radius"]["right_wheel"] = -wheel_radius
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
