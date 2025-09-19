#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2025 Inria
#
# /// script
# dependencies = ["upkie", "pybullet>=3"]
# ///

"""Show inertia randomization in PyBullet simulation."""

import gymnasium as gym
import numpy as np
from scipy.spatial.transform import Rotation as ScipyRotation

import upkie.envs
from upkie.utils.robot_state import RobotState

EPISODE_DURATION = 5.0  # seconds
CONTROL_FREQUENCY = 200.0  # Hz
EPISODE_STEPS = int(EPISODE_DURATION * CONTROL_FREQUENCY)

# Different inertia variation levels to try out
INERTIA_VARIATIONS = [0.0, 0.1, 0.3]


def run_episode(env, episode_name: str):
    """Run a single episode with the given environment."""
    print(f"\n=== {episode_name} ===")

    observation, info = env.reset()
    print(f"Initial pitch: {observation[0]:.3f} rad")

    for step in range(EPISODE_STEPS):
        # Simple PD balancer
        pitch = observation[0]
        ground_position = observation[1]
        ground_velocity = observation[3]
        v = 10.0 * pitch + 1.0 * ground_position + 0.1 * ground_velocity
        action = np.clip([v], -0.9, 0.9)
        observation, reward, terminated, truncated, info = env.step(action)

        if terminated or truncated:
            observation, info = env.reset()

        # Print some diagnostic info every second
        if step % int(CONTROL_FREQUENCY) == 0:
            # Also show how inertia variations affect joint torques
            spine_obs = info["spine_observation"]
            left_hip_torque = spine_obs["servo"]["left_hip"]["torque"]
            left_knee_torque = spine_obs["servo"]["left_knee"]["torque"]
            print(
                f"  Step {step:4d}: "
                f"pitch={observation[0]:.3f} rad, "
                f"ground_vel={observation[3]:.3f} m/s, "
                f"hip_torque={left_hip_torque:.3f} N⋅m, "
                f"knee_torque={left_knee_torque:.3f} N⋅m"
            )

    print(f"Final pitch: {observation[0]:.3f} rad")


if __name__ == "__main__":
    upkie.envs.register()

    print("Inertia Randomization Example")
    print("=============================\n")
    print("This example shows how inertia randomization affects dynamics.")
    print("We run the same controller with increasing inertia randomization.")
    print("Notice how the robot's behavior changes progressively.")

    for i, inertia_variation in enumerate(INERTIA_VARIATIONS):
        # Configure bullet with inertia variation
        bullet_config = {
            "inertia_variation": inertia_variation,
        }

        # Create environment with current configuration (GUI disabled for CI)
        with gym.make(
            "Upkie-PyBullet-Pendulum",
            frequency=CONTROL_FREQUENCY,
            gui=True,
            init_state=RobotState(
                position_base_in_world=np.array([0.0, 0.0, 0.6]),
                orientation_base_in_world=ScipyRotation.from_euler(
                    "xyz", [0.0, 0.1, 0.0]
                ),
            ),
            bullet_config=bullet_config,
        ) as env:
            run_episode(
                env,
                episode_name=(
                    f"Episode {i + 1}: "
                    f"inertias randomized by ±{inertia_variation * 100:.0f}%"
                ),
            )

    print("\n" + "=" * 60)
    print("Example completed!")
    print("\nObservations:")
    print("- Model inertias: Deterministic, repeatable behavior")
    print("- Small inertia variations: Subtle changes in dynamics")
    print("- Large inertia variations: More noticeable differences")
    print("\nThis feature is useful for:")
    print("- Domain randomization in reinforcement learning")
    print("- Testing controller performance across robot model variations")
