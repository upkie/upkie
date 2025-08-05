#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

"""Wheel balancing using model predictive control of an LTV system."""

import gymnasium as gym
import numpy as np

import upkie.envs
from upkie.controllers import MPCBalancer

upkie.envs.register()

TARGET_GROUND_VELOCITY = 0.0  # m/s


def select_gym_environment():
    """Prompt user to choose between simulation environments."""
    print("Select your Gymnasium environment:")
    print(
        "1. Upkie-Spine-Pendulum      \t"
        "requires a running spine (e.g. from ./start_simulation.sh)"
    )
    print("2. Upkie-PyBullet-Pendulum \tself-contained PyBullet simulation")

    while True:
        try:
            choice = input("Enter your choice (1 or 2): ").strip()
            if choice == "1":
                return "Upkie-Spine-Pendulum", {"frequency": 200.0}
            elif choice == "2":
                return "Upkie-PyBullet-Pendulum", {
                    "frequency": 200.0,
                    "nb_substeps": 5,
                }
            else:
                print("Invalid choice. Please enter 1 or 2.")
        except KeyboardInterrupt:
            exit(0)


if __name__ == "__main__":
    mpc_balancer = MPCBalancer()
    env_name, env_kwargs = select_gym_environment()
    print(f"\nStarting MPC balancing with {env_name}...")

    try:
        with gym.make(env_name, **env_kwargs) as env:
            _, info = env.reset()  # connects to the spine
            action = np.zeros(env.action_space.shape)

            print("Running MPC balancing for 10,000 steps...")
            print(f"Target ground velocity: {TARGET_GROUND_VELOCITY} m/s")
            print("Press Ctrl+C to stop early.\n")

            for step in range(10_000):
                action[0] = mpc_balancer.compute_ground_velocity(
                    target_ground_velocity=TARGET_GROUND_VELOCITY,  # m/s
                    spine_observation=info["spine_observation"],
                    dt=env.unwrapped.dt,
                )
                _, _, terminated, truncated, info = env.step(action)

                # Print progress every 1000 steps
                if (step + 1) % 1000 == 0:
                    ground_pos = info["spine_observation"]["wheel_odometry"][
                        "position"
                    ]
                    base_pitch = info["spine_observation"]["base_orientation"][
                        "pitch"
                    ]
                    print(
                        f"Step {step + 1:5d}: "
                        f"Ground position = {ground_pos:6.3f} m, "
                        f"Base pitch = {base_pitch:6.3f} rad"
                    )

                if terminated or truncated:
                    print("Episode ended, resetting...")
                    _, info = env.reset()

            print("Example completed.")

    except KeyboardInterrupt:
        print("\nStopped by user.")
    except Exception as e:
        if "spine" in str(e).lower():
            print(f"\nConnection error: {e}")
            print(
                "Make sure a spine running, for instance "
                "a simulation started with: ./start_simulation.sh"
            )
        else:
            print(f"\nError: {e}")
        exit(1)
