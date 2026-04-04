#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
#
# /// script
# dependencies = ["upkie", "proxsuite", "pybullet", "qpmpc"]
# ///

"""Follow a 1 m square using the UpkieBaseVelocity environment in PyBullet."""

import argparse
import math

import gymnasium as gym
import numpy as np

import upkie.envs

upkie.envs.register()

SIDE_LENGTH = 1.0  # meters
LINEAR_VELOCITY = 0.3  # m/s
YAW_VELOCITY = 0.5  # rad/s


def main(
    robot: str,
    gain_scale: float = 2.0,
    turning_gain_scale: float = 2.0,
):
    print(f"\nFollowing a ground square with {robot} in PyBullet...")
    with gym.make(
        f"{robot}-PyBullet-BaseVelocity",
        frequency=200.0,
        frequency_checks=False,
        gui=True,
        nb_substeps=5,
    ) as env:
        observation, _ = env.reset()
        set_leg_gain_scale = env.get_wrapper_attr("set_leg_gain_scale")

        square_side = 0
        target_yaw = 0.0
        side_start_x, side_start_y = 0.0, 0.0
        turning = False
        finished = False

        print(
            f"\nFollowing a {SIDE_LENGTH} m square at "
            f"{LINEAR_VELOCITY} m/s..."
        )
        print("Press Ctrl+C to stop.\n")

        while square_side <= 4:
            x, y, yaw = observation

            if square_side == 4:
                action = np.array([0.0, 0.0], dtype=np.float32)
            elif not turning:
                # Go forward along current heading
                set_leg_gain_scale(gain_scale)
                action = np.array([LINEAR_VELOCITY, 0.0], dtype=np.float32)
                dx = x - side_start_x
                dy = y - side_start_y
                distance = math.hypot(dx, dy)
                if distance >= SIDE_LENGTH:
                    turning = True
                    target_yaw += math.pi / 2
                    print(
                        f"  Side {square_side + 1} done at "
                        f"({x:.2f}, {y:.2f}), "
                        f"turning to "
                        f"{math.degrees(target_yaw):.0f} deg"
                    )
            else:
                set_leg_gain_scale(gain_scale + turning_gain_scale)
                # Turn left until we reach target yaw
                yaw_error = target_yaw - yaw
                if abs(yaw_error) < 0.02:
                    turning = False
                    square_side += 1
                    side_start_x, side_start_y = x, y
                    print(
                        f"  Turn done, " f"yaw = {math.degrees(yaw):.1f} deg"
                    )
                    continue
                action = np.array([0.0, YAW_VELOCITY], dtype=np.float32)

            observation, _, terminated, truncated, _ = env.step(action)

            if terminated or truncated:
                print("Episode ended, resetting...")
                observation, _ = env.reset()
                square_side = 0
                target_yaw = 0.0
                side_start_x, side_start_y = 0.0, 0.0
                turning = False
                finished = False
            elif not finished and square_side == 4:
                x, y, yaw = observation
                print(
                    f"\nSquare completed! "
                    f"Final position: ({x:.2f}, {y:.2f}), "
                    f"yaw = {math.degrees(yaw):.1f} deg"
                )
                finished = True


def pick_upkie_description() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--robot",
        help="Robot to simulate (default: upkie)",
        choices=["upkie", "cookie"],
        default="upkie",
    )
    return parser.parse_args().robot.capitalize()


if __name__ == "__main__":
    try:
        main(pick_upkie_description())
    except KeyboardInterrupt:
        print("Terminating in response to keyboard interrupt")
