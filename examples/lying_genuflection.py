#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria
# SPDX-License-Identifier: Apache-2.0

"""Genuflect while lying on a horizontal floor."""

import gymnasium as gym
import numpy as np

import upkie.envs

NB_GENUFLECTIONS = 10
GENUFLECTION_STEPS = 200
AMPLITUDE = 1.0  # in radians

LEFT_HIP = 0  # index of left hip position in vectorized action
LEFT_KNEE = 1  # same for left knee, etc.
RIGHT_HIP = 3
RIGHT_KNEE = 4

spine_config = {
    "bullet": {
        "reset": {
            "orientation_base_in_world": [0.707, 0.0, -0.707, 0.0],
            "position_base_in_world": [0.0, 0.0, 0.1],
        },
    }
}

if __name__ == "__main__":
    upkie.envs.register()
    with gym.make(
        "UpkieServos-v3",
        spine_config=spine_config,
        frequency=200.0,
    ) as env:
        action = env.get_default_action()
        observation, _ = env.reset()  # connects to the spine
        for step in range(NB_GENUFLECTIONS * GENUFLECTION_STEPS):
            x = float(step % GENUFLECTION_STEPS) / GENUFLECTION_STEPS
            y = 4.0 * x * (1.0 - x)  # in [0, 1]
            q_0134 = AMPLITUDE * y * np.array([1.0, -2.0, -1.0, 2.0])
            action["left_hip"]["position"] = q_0134[0]
            action["left_knee"]["position"] = q_0134[1]
            action["right_hip"]["position"] = q_0134[2]
            action["right_knee"]["position"] = q_0134[3]
            observation, _, _, _, _ = env.step(action)
