#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria
#
# /// script
# dependencies = ["upkie"]
# ///

"""Wheel balancing using model predictive control with a running spine."""

import gymnasium as gym
import numpy as np

import upkie.envs
from upkie.controllers import MPCBalancer

upkie.envs.register()

NB_STEPS = 5_000
TARGET_GROUND_VELOCITY = 0.3  # m/s


if __name__ == "__main__":
    # Leg length is a key parameter of the model predictive controller. Start
    # from the leg length of the wheeled biped, but don't hesitate to tune this
    # value afterwards for better performance on your real robot.
    mpc_balancer = MPCBalancer(leg_length=0.58)

    try:
        env_kwargs = {
            "frequency": 200.0,
            "frequency_checks": False,  # simulation steps run within env steps
            "gui": True,
        }

        print("\nStarting MPC balancing with Upkie-Spine-Pendulum...")
        print("Make sure a spine is running, e.g. from ./start_simulation.sh")
        with gym.make("Upkie-Spine-Pendulum", **env_kwargs) as env:
            _, info = env.reset()  # connects to the spine
            action = np.zeros(env.action_space.shape)

            print("\n--\n")
            print(f"Running MPC balancing for {NB_STEPS} steps...")
            print(f"Target ground velocity: {TARGET_GROUND_VELOCITY} m/s")
            print("Press Ctrl+C to stop early.\n")

            for step in range(NB_STEPS):
                action[0] = mpc_balancer.compute_ground_velocity(
                    target_ground_velocity=TARGET_GROUND_VELOCITY,  # m/s
                    spine_observation=info["spine_observation"],
                    dt=env.unwrapped.dt,
                )
                _, _, terminated, truncated, info = env.step(action)

                # Print progress every 1000 steps
                if (step + 1) % (NB_STEPS // 10) == 0:
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
                "Make sure a spine is running, for instance "
                "a simulation started with: ./start_simulation.sh"
            )
        else:
            print(f"\nError: {e}")
        exit(1)
