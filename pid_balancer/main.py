#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron
# Copyright 2023 Inria

import argparse
import socket
import traceback
from os import path

import gin
import yaml
from loop_rate_limiters import RateLimiter
from upkie.utils.raspi import configure_agent_process, on_raspi
from upkie.utils.spdlog import logging
from vulp.spine import SpineInterface

from servo_controller import ServoController


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
    )
    return parser.parse_args()


def run(
    spine: SpineInterface,
    spine_config: dict,
    frequency: float = 200.0,
) -> None:
    r"""!
    Read observations and send actions to the spine.

    \param spine Interface to the spine.
    \param spine_config Spine configuration dictionary.
    \param frequency Control frequency in Hz.
    """
    controller = ServoController()
    dt = 1.0 / frequency
    rate = RateLimiter(frequency, "controller")

    wheel_radius = controller.wheel_radius
    spine_config["bullet"]["monitor"] = {
        "contacts": {"left_wheel_tire": True, "right_wheel_tire": True}
    }
    spine_config["wheel_odometry"] = {
        "signed_radius": {
            "left_wheel": +wheel_radius,
            "right_wheel": -wheel_radius,
        },
    }

    spine.start(spine_config)
    observation = spine.get_observation()  # pre-reset observation
    while True:
        observation = spine.get_observation()
        action = controller.cycle(observation, dt)
        action["env"] = {"rate": {"slack": rate.slack}}
        spine.set_action(action)
        rate.sleep()


def load_gin_configuration(base_dir: str, path: str) -> None:
    logging.info(f"Loading configuration '{path}'")
    try:
        gin.parse_config_file(f"{base_dir}/{path}")
    except OSError as e:
        raise FileNotFoundError(f"Configuration '{path}' not found") from e


if __name__ == "__main__":
    if on_raspi():
        configure_agent_process()

    args = parse_command_line_arguments()
    agent_dir = path.dirname(__file__)

    # Load gin configuration files
    load_gin_configuration(agent_dir, "config/common.gin")
    hostname = socket.gethostname().lower()
    if args.config is None:
        logging.warning(f"No configuration selected, trying '{hostname}'")
        load_gin_configuration(agent_dir, f"config/{hostname}.gin")
    elif args.config == "hostname":
        logging.info(f"Loading configuration from hostname '{hostname}'")
        load_gin_configuration(agent_dir, f"config/{hostname}.gin")
    elif args.config is not None:
        load_gin_configuration(agent_dir, f"config/{args.config}.gin")

    # Spine configuration
    with open(f"{agent_dir}/config/spine.yaml", "r") as fh:
        spine_config = yaml.safe_load(fh)

    spine = SpineInterface()
    try:
        run(spine, spine_config)
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
