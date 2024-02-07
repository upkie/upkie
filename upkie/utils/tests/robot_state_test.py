#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

"""Test robot state and its randomization."""

import unittest

import numpy as np

from upkie.utils.robot_state import RobotState
from upkie.utils.robot_state_randomization import RobotStateRandomization


class TestRobotState(unittest.TestCase):
    def test_robot_state(self):
        init_state = RobotState(
            randomization=RobotStateRandomization(roll=0.0, pitch=0.0)
        )
        zyx_angles = init_state.sample_orientation(np.random).as_euler("ZYX")
        self.assertTrue(np.allclose(zyx_angles, np.zeros(3)))


if __name__ == "__main__":
    unittest.main()
