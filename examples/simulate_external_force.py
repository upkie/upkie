#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

"""Lift the simulated robot while it balances in place."""

import gymnasium as gym
import numpy as np

import upkie.envs

upkie.envs.register()


def get_vertical_force(
    step: int,
    start: int = 400,
    lift_steps: int = 400,
    delta: float = 0.5,
    hold_steps: int = 200,
) -> float:
    lift: float = 0.0  # no unit
    if step < start:
        lift = 0.0
    elif step - start < lift_steps // 2:
        lift = 1 + delta
    elif step - start < lift_steps:
        lift = 1 - delta
    elif step - start - lift_steps < hold_steps:
        lift = 1.0
    else:
        lift = 0.0
    mass = 5.375  # [kg]
    return lift * mass * 9.81  # in [N]


def run(env: upkie.envs.UpkieGroundVelocity):
    torso_force_in_world = np.zeros(3)
    bullet_action = {
        "external_forces": {
            "torso": {
                "force": torso_force_in_world,
                "local": False,
            }
        }
    }
    observation, _ = env.reset()
    gain = np.array([10.0, 1.0, 0.0, 0.1])
    for step in range(1_000_000):
        action = gain.dot(observation).reshape((1,))
        torso_force_in_world[2] = get_vertical_force(step % 1000)
        env.bullet_extra(bullet_action)  # call before env.step
        observation, _, terminated, truncated, _ = env.step(action)
        if terminated or truncated:
            observation, _ = env.reset()


if __name__ == "__main__":
    with gym.make("UpkieGroundVelocity-v3", frequency=200.0) as env:
        run(env)
