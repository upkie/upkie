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


def run(env: UpkiePendulum):
    torso_force_in_world = np.zeros(3)
    gain = np.array([10.0, 1.0, 0.0, 0.1])
    simulator = env.unwrapped.backend

    observation, _ = env.reset()
    for step in range(1_000_000):
        action = gain.dot(observation).reshape((1,))
        action = np.clip(action, -0.9, 0.9)

        # Lift the robot periodically
        torso_force_in_world[2] = get_lifting_force(step % 1000)
        if torso_force_in_world[2] > 1.0:
            action *= 0.0

        # Apply the external force directly in the PyBullet backend
        external_forces = {
            "torso": {
                "force": torso_force_in_world,
                "local": False,
            }
        }
        simulator.set_external_forces(external_forces)

        observation, _, terminated, truncated, _ = env.step(action)
        if terminated or truncated:
            observation, _ = env.reset()


if __name__ == "__main__":
    with gym.make("Upkie-PyBullet-Pendulum", frequency=200.0, gui=True) as env:
        run(env)
