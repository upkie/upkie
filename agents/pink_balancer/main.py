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
from agents.pink_balancer.whole_body_controller import WholeBodyController
from utils.realtime import configure_cpu
from vulp.spine import SpineInterface

config = {
    "bullet": {
        "control_mode": "torque",
        "follower_camera": False,
        "gui": True,
        "position_init_base_in_world": [0.0, 0.0, 0.6],
        "torque_control": {
            "kp": 20.0,
            "kd": 1.0,
        },
    },
    "cpu": 3,
    "floor_contact": {
        "upper_leg_torque_threshold": 10.0,
    },
    "servo_layout": {
        "left_hip": {
            "bus": 2,
            "configuration_index": 7,
            "id": 4,
        },
        "left_knee": {
            "bus": 2,
            "configuration_index": 8,
            "id": 5,
        },
        "left_wheel": {
            "bus": 2,
            "id": 6,
        },
        "right_hip": {
            "bus": 1,
            "configuration_index": 10,
            "id": 1,
        },
        "right_knee": {
            "bus": 1,
            "configuration_index": 11,
            "id": 2,
        },
        "right_wheel": {
            "bus": 1,
            "id": 3,
        },
    },
    "upkie": {
        "brain": {
            "joystick": {
                "crouch_velocity": 0.02,
            },
        },
        "height": 0.30,
    },
    "wheel_contact": {
        "cutoff_period": 0.2,
        "liftoff_inertia": 0.001,
        "min_touchdown_acceleration": 2.0,
        "min_touchdown_torque": 0.015,
        "touchdown_inertia": 0.004,
    },
    "wheel_odometry": {
        "signed_radius": {
            "left_wheel": +0.06,
            "right_wheel": -0.06,
        },
    },
}


def parse_command_line_arguments():
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
    gin.parse_config_file(f"{agent_dir}/whole_body_controller.gin")
    gin.parse_config_file(f"{agent_dir}/wheel_balancer.gin")
    if args.config == "default":
        logging.warn('No configuration specified, assuming "bullet"')
        args.config = "bullet"
    if args.config == "pi3hat":
        gin.parse_config_file(f"{agent_dir}/pi3hat.gin")
    elif args.config == "bullet":
        gin.parse_config_file(f"{agent_dir}/bullet.gin")

    config_dir = f"{agent_dir}/../../config"
    if args.config == "pi3hat":
        configure_cpu(config["cpu"])

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
