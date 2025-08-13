#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

"""Initialize servo environment with a custom initial state."""

import gymnasium as gym
import numpy as np
from scipy.spatial.transform import Rotation as ScipyRotation

import upkie.envs
from upkie.utils.robot_state import RobotState

if __name__ == "__main__":
    upkie.envs.register()
    with gym.make(
        "Upkie-Spine-Servos",
        frequency=200.0,
        init_state=RobotState(
            orientation_base_in_world=ScipyRotation.from_quat(
                [0.0, 0.0, -0.707, 0.707],  # SciPy convention: (x, y, z, w))
            ),
            position_base_in_world=np.array([0.1, 0.0, 0.7]),
            joint_configuration=np.array([1.0, 2.0, 3.0, 0.0, 0.0, 0.0]),
        ),
    ) as env:
        for step in range(10_000):
            env.reset()  # keep resetting to show the initial state
