#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria
#
# /// script
# dependencies = ["upkie", "pybullet>=3"]
# ///

"""Lift the simulated robot while it balances in place."""

import gymnasium as gym
import numpy as np

import upkie.envs
from upkie.envs.upkie_pendulum import UpkiePendulum
from upkie.utils.external_force import ExternalForce

upkie.envs.register()


def get_lifting_force(
    step: int,
    start: int = 200,
    lift_steps: int = 200,
    delta: float = 0.1,
    hold_steps: int = 400,
) -> float:
    lift: float = 0.0  # 0 = no force, 1 => apply -mg
    if step < start:
        lift = 0.0
    elif step - start < lift_steps // 2:
        lift = 1.0 + delta
    elif step - start < lift_steps:
        lift = 1.0 - delta
    elif step - start - lift_steps < hold_steps:
        lift = 1.0
    else:
        lift = 0.0
    mass = 5.34  # approximative, in [kg]
    return lift * mass * 9.81  # in [N]


def run(env: UpkiePendulum, nb_steps: int = 5000):
    torso_force_in_world = np.zeros(3)
    gain = np.array([10.0, 1.0, 0.0, 0.1])
    simulator = env.unwrapped.backend

    print("Upkie balancing simulation with periodic lifting")
    print("Watch the GUI to see the external force effects!")
    print(f"Running for {nb_steps} steps...")

    observation, _ = env.reset()
    for step in range(nb_steps):
        action = gain.dot(observation).reshape((1,))
        action = np.clip(action, -0.9, 0.9)

        # Lift the robot periodically
        cycle_step = step % 1000
        torso_force_in_world[2] = get_lifting_force(cycle_step)

        # Comment for user
        if cycle_step == 0:
            print(f"⚖️  Step {step:4d}: No external force")
        elif cycle_step == 200:
            print(
                f"⬆️  Step {step:4d}: Applying upward force at "
                f"{torso_force_in_world[2]:.1f} N"
            )
        elif cycle_step == 300:
            print(
                f"⬇️  Step {step:4d}: Reducing upward force "
                f"to {torso_force_in_world[2]:.1f} N"
            )
        elif cycle_step == 400:
            print(
                f"🔒 Step {step:4d}: Holding robot in the air "
                f"with an upward force of {torso_force_in_world[2]:.1f} N"
            )
        elif cycle_step == 800:
            print(f"📉 Step {step:4d}: Lowering robot back to ground")

        if torso_force_in_world[2] > 1.0:
            action *= 0.0

        # Set external forces using the new ExternalForce class
        external_forces = {
            "torso": ExternalForce(
                force=torso_force_in_world,
                local=False,
            )
        }
        simulator.set_external_forces(external_forces)

        # External forces will be applied during env.step
        observation, _, terminated, truncated, _ = env.step(action)
        if terminated or truncated:
            print("🔄 Environment reset, robot fell or triggered termination")
            observation, _ = env.reset()


if __name__ == "__main__":
    with gym.make("Upkie-PyBullet-Pendulum", frequency=200.0, gui=True) as env:
        run(env)
