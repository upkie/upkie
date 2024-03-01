#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

"""This example simply makes Upkie go to its neutral configuration."""

import gymnasium as gym
import upkie.envs

upkie.envs.register()

RESET_DURATION = 2.0  # seconds
HOLD_DURATION = 10.0  # seconds

JOINTS = [
    f"{side}_{name}"
    for side in ("left", "right")
    for name in ("hip", "knee", "wheel")
]


def reset_to_neutral(env: upkie.envs.UpkieServos):
    action = env.unwrapped.get_neutral_action()
    observation, _ = env.reset()  # connects to the spine

    for joint in JOINTS:
        position = observation["servo"][joint]["position"]
        action[joint]["position"] = position
        action[joint]["velocity"] = -position / RESET_DURATION

    nb_steps = int(RESET_DURATION / env.dt)
    for _ in range(nb_steps):
        observation, _, _, _, _ = env.step(action)
        for joint in JOINTS:
            action[joint]["position"] += action[joint]["velocity"] * env.dt

    for joint in JOINTS:
        action[joint]["position"] = 0.0
        action[joint]["velocity"] = 0.0
    return observation, action


if __name__ == "__main__":
    with gym.make("UpkieServos-v3", frequency=200.0) as env:
        observation, action = reset_to_neutral(env)
        nb_steps = int(HOLD_DURATION / env.dt)
        for step in range(nb_steps):
            observation, _, _, _, _ = env.step(action)
