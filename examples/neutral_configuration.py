#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

"""This example simply makes Upkie go to its neutral configuration."""

import gymnasium as gym

import upkie.envs

upkie.envs.register()

RESET_DURATION = 10.0  # seconds


def reset_to_neutral(env: upkie.envs.UpkieServos):
    JOINTS = [
        f"{side}_{name}"
        for side in ("left", "right")
        for name in ("hip", "knee", "wheel")
    ]

    action = env.unwrapped.get_neutral_action()
    observation, _ = env.reset()  # connects to the spine
    for joint in JOINTS:
        position = observation[joint]["position"]
        action[joint]["position"] = position
        action[joint]["velocity"] = -position / RESET_DURATION

    dt = env.unwrapped.dt
    nb_steps = int(RESET_DURATION / dt)
    for _ in range(nb_steps):
        env.step(action)
        for joint in JOINTS:
            action[joint]["position"] += action[joint]["velocity"] * dt

    for joint in JOINTS:
        action[joint]["position"] = 0.0
        action[joint]["velocity"] = 0.0
    return action


if __name__ == "__main__":
    with gym.make("Upkie-Spine-Servos", frequency=200.0) as env:
        last_action = reset_to_neutral(env)
        while True:  # hold configuration until the agent is interrupted
            env.step(last_action)
