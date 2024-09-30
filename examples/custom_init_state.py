#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

"""Genuflect while lying on a horizontal floor."""

import gymnasium as gym
import numpy as np
from scipy.spatial.transform import Rotation as ScipyRotation

import upkie.envs
from upkie.utils.robot_state import RobotState

NB_GENUFLECTIONS = 10
GENUFLECTION_STEPS = 200
AMPLITUDE = 1.0  # in radians

if __name__ == "__main__":
    upkie.envs.register()
    with gym.make(
        "UpkieServoPositions-v4",
        frequency=200.0,
        init_state=RobotState(
            orientation_base_in_world=ScipyRotation.from_quat(
                [0.0, -0.707, 0.0, 0.707]  # SciPy convention: (x, y, z, w)
            ),
            position_base_in_world=np.array([0.0, 0.0, 0.1]),
            joint_configuration=np.array([1.0, 2.0, 3.0, 0.0, 0.0, 0.0]),
        ),
    ) as env:
        action = env.get_neutral_action()
        env.reset()  # connects to the spine
        for step in range(NB_GENUFLECTIONS * GENUFLECTION_STEPS):
            x = float(step % GENUFLECTION_STEPS) / GENUFLECTION_STEPS
            y = 4.0 * x * (1.0 - x)  # in [0, 1]
            q_0134 = AMPLITUDE * y * np.array([1.0, -2.0, -1.0, 2.0])
            action["left_hip"]["position"] = q_0134[0]
            action["left_knee"]["position"] = q_0134[1]
            action["right_hip"]["position"] = q_0134[2]
            action["right_knee"]["position"] = q_0134[3]
            # env.step(action)
            env.reset()
