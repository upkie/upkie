#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Upkie Team
#
# /// script
# dependencies = ["upkie", "proxsuite", "pybullet>=3", "qpmdc"]
# ///

"""Navigation example using UpkieNavigation in PyBullet simulation."""

import gymnasium as gym
import numpy as np

import upkie.envs

upkie.envs.register()

NB_STEPS = 8_000
SIDE_LENGTH = 0.5  # meters (50 cm)


def follow_square_trajectory(step_count: int, total_steps: int) -> np.ndarray:
    steps_per_side = total_steps // 4
    step_in_side = step_count % steps_per_side
    linear_velocity = 0.2  # m/s
    angular_velocity = 0.5  # rad/s
    frequency = 200.0
    dt = 1.0 / frequency

    forward_steps = int(SIDE_LENGTH / linear_velocity / dt)  # 200 Hz
    if step_in_side < forward_steps:
        return np.array([linear_velocity, 0.0])
    else:
        # Turn left 90 degrees
        return np.array([0.0, angular_velocity])


if __name__ == "__main__":
    with gym.make(
        "Upkie-PyBullet-Navigation",
        frequency=200.0,  # Hz
        frequency_checks=False,  # no real-time check as simulator runs in step
        gui=True,
        nb_substeps=5,
        max_linear_velocity=1.0,  # m/s
        max_angular_velocity=2.0,  # rad/s
    ) as env:
        _, info = env.reset()  # connects to the spine

        print("\n-----\n")
        print(f"Tracking a square for {NB_STEPS} steps...")
        print(f"Square side: {SIDE_LENGTH} m")
        print("Press Ctrl+C to stop early.\n")

        for step in range(NB_STEPS):
            action = follow_square_trajectory(step, NB_STEPS)
            observation, _, terminated, truncated, info = env.step(action)

            # Print progress every 500 steps
            if (step + 1) % 500 == 0:
                x, y, theta = observation
                print(
                    f"Step {step + 1:5d}: "
                    f"Position = ({x:6.3f}, {y:6.3f}) m, "
                    f"Orientation = {theta:6.3f} rad "
                    f"({np.degrees(theta):6.1f}°)"
                )

            if terminated or truncated:
                print("Episode ended, resetting...")
                _, info = env.reset()

        print("Navigation example completed.")
