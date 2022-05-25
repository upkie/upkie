#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2022 Stéphane Caron
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import asyncio
import logging
import traceback
from os import path
from typing import Any, Dict

import aiorate
import gin
import yaml
from agents.blue_balancer.whole_body_controller import WholeBodyController
from utils.realtime import configure_cpu
from vulp.spine import SpineInterface


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
        default="default",
        type=str,
        required=False,
        choices=["default", "bullet", "pi3hat"],
    )
    return parser.parse_args()


async def run(
    spine: SpineInterface,
    config: Dict[str, Any],
    frequency: float = 200.0,
) -> None:
    """
    Read observations and send actions to the spine.

    Args:
        spine: Interface to the spine.
        config: Configuration dictionary.
        frequency: Control frequency in Hz.
    """
    whole_body_controller = WholeBodyController(config)
    dt = 1.0 / frequency
    rate = aiorate.Rate(frequency, "controller")
    spine.start(config)
    observation = spine.get_observation()  # pre-reset observation
    while True:
        observation = spine.get_observation()
        action = whole_body_controller.cycle(observation, dt)
        spine.set_action(action)
        await rate.sleep()


if __name__ == "__main__":
    args = parse_command_line_arguments()
    agent_dir = path.dirname(__file__)

    # Gin configuration
    gin.parse_config_file(f"{agent_dir}/whole_body_controller.gin")
    gin.parse_config_file(f"{agent_dir}/wheel_balancer.gin")
    if args.config == "default":
        logging.warning('No configuration specified, assuming "bullet"')
        args.config = "bullet"
    if args.config == "pi3hat":
        gin.parse_config_file(f"{agent_dir}/pi3hat.gin")
    elif args.config == "bullet":
        gin.parse_config_file(f"{agent_dir}/bullet.gin")

    # Spine configuration
    with open(f"{agent_dir}/spine.yaml", "r") as fh:
        config = yaml.safe_load(fh)
    if args.config == "pi3hat":
        configure_cpu(cpu=3)

    spine = SpineInterface()
    try:
        asyncio.run(run(spine, config))
    except KeyboardInterrupt:
        logging.info("Caught a keyboard interrupt")
    except Exception:
        logging.error("Controller raised an exception")
        print("")
        traceback.print_exc()
        print("")

    logging.info("Stopping the spine")
    try:
        spine.stop()
    except Exception:
        logging.error("Error while stopping the spine!")
        print("")
        traceback.print_exc()
        print("")
