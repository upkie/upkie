#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
#
# /// script
# dependencies = ["upkie", "proxsuite", "pybullet", "qpmpc"]
# ///

"""Follow joystick commands in simulation for Upkie or Cookie."""

import argparse

import gymnasium as gym
import numpy as np

import upkie.envs
from upkie.controllers import JoystickGyropodController
from upkie.utils.clamp import clamp

upkie.envs.register()


def main(
    robot: str = "Upkie",
    gain_scale: float = 2.0,
    turning_gain_scale: float = 2.0,
) -> None:
    gain_scale = clamp(gain_scale, 0.1, 2.0)
    joystick_controller = JoystickGyropodController()
    turning_gain_scale = clamp(turning_gain_scale, 0.0, 3.0)

    with gym.make(
        f"{robot}-PyBullet-BaseVelocity",
        frequency=200.0,
        gui=True,
    ) as env:
        dt = env.unwrapped.dt
        _, info = env.reset()
        spine_observation = info["spine_observation"]
        set_leg_gain_scale = env.get_wrapper_attr("set_leg_gain_scale")

        while True:
            linear_velocity, yaw_velocity = joystick_controller.step(
                spine_observation, dt
            )

            # Update leg gain scaling based on turning probability
            turning_prob = joystick_controller.turning_probability
            set_leg_gain_scale(gain_scale + turning_gain_scale * turning_prob)

            action = np.array([linear_velocity, yaw_velocity])
            _, _, terminated, truncated, info = env.step(action)
            spine_observation = info["spine_observation"]

            if terminated or truncated:
                _, info = env.reset()
                spine_observation = info["spine_observation"]
                joystick_controller.reset()


def parse_command_line_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--robot",
        help="Robot to simulate (default: upkie)",
        choices=["upkie", "cookie"],
        default="upkie",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_command_line_arguments()
    robot = args.robot.capitalize()
    try:
        main(robot)
    except KeyboardInterrupt:
        print("Terminating in response to keyboard interrupt")
