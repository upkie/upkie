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
import datetime
import os
import shutil
import socket
import time
import traceback
from os import path
from typing import Any, Dict

import gin
import mpacklog
import yaml
from loop_rate_limiters import AsyncRateLimiter
from vulp.spine import SpineInterface

from agents.pink_balancer.whole_body_controller import WholeBodyController
from utils.realtime import configure_cpu
from utils.spdlog import logging


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


async def run(
    spine: SpineInterface,
    spine_config: Dict[str, Any],
    logger: mpacklog.Logger,
    args: argparse.Namespace,
    frequency: float = 200.0,
) -> None:
    """
    Read observations and send actions to the spine.

    Args:
        spine: Interface to the spine.
        config: Configuration dictionary.
        frequency: Control frequency in Hz.
    """
    controller = WholeBodyController(spine_config, visualize=args.visualize)
    debug: Dict[str, Any] = {}
    dt = 1.0 / frequency
    rate = AsyncRateLimiter(frequency, "controller")

    wheel_radius = controller.wheel_balancer.wheel_radius
    spine_config["wheel_odometry"] = {
        "signed_radius": {
            "left_wheel": +wheel_radius,
            "right_wheel": -wheel_radius,
        }
    }

    spine.start(spine_config)
    observation = spine.get_observation()  # pre-reset observation
    while True:
        observation = spine.get_observation()
        action = controller.cycle(observation, dt)
        action_time = time.time()
        spine.set_action(action)
        debug["rate"] = {
            "measured_period": rate.measured_period,
            "slack": rate.slack,
        }
        await logger.put(
            {
                "action": action,
                "debug": debug,
                "observation": observation,
                "time": action_time,
            }
        )
        await rate.sleep()


async def main(spine, config: Dict[str, Any], args: argparse.Namespace):
    logger = mpacklog.Logger("/dev/shm/brain.mpack")
    await logger.put(
        {
            "config": config,
            "time": time.time(),
        }
    )
    await asyncio.gather(
        run(spine, config, logger, args),
        logger.write(),
        return_exceptions=False,  # make sure exceptions are raised
    )


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

    # Spine configuration
    with open(f"{agent_dir}/config/spine.yaml", "r") as fh:
        spine_config = yaml.safe_load(fh)
    if args.config == "pi3hat":
        configure_cpu(cpu=3)

    spine = SpineInterface()
    try:
        asyncio.run(main(spine, spine_config, args))
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

    now = datetime.datetime.now()
    stamp = now.strftime("%Y-%m-%d_%H%M%S")
    log_dir = os.environ.get("UPKIE_LOG_PATH", "~")
    save_path = os.path.expanduser(f"{log_dir}/{stamp}_pink_balancer.mpack")
    shutil.copy("/dev/shm/brain.mpack", save_path)
    logging.info(f"Log saved to {save_path}")
