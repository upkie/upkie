#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

"""Tests for UpkieServoPositions environment."""

import unittest
from multiprocessing.shared_memory import SharedMemory

import numpy as np

from upkie.envs import UpkieServoPositions
from upkie.envs.tests.mock_spine import MockSpine


class TestUpkieServoPositions(unittest.TestCase):
    def setUp(self):
        shared_memory = SharedMemory(name=None, size=42, create=True)
        self.env = UpkieServoPositions(
            frequency=100.0,
            shm_name=shared_memory._name,
        )
        shared_memory.close()
        self.env._spine = MockSpine()

    def test_action_clamping(self):
        self.env.reset()
        action = {
            joint.name: {
                "position": np.nan,
            }
            for joint in self.env.model.joints
        }

        action["left_hip"]["position"] = np.nan
        self.env.step(action)
        self.assertTrue(
            np.isnan(self.env._spine.action["servo"]["left_hip"]["position"])
        )

        action["left_hip"]["position"] = 0.5
        self.env.step(action)
        self.assertAlmostEqual(
            self.env._spine.action["servo"]["left_hip"]["position"],
            0.5,
            places=5,
        )

        action["left_hip"]["position"] = 5e5
        self.env.step(action)
        self.assertAlmostEqual(
            self.env._spine.action["servo"]["left_hip"]["position"],
            self.env.action_space["left_hip"]["position"].high[0],
            places=5,
        )

    def test_action_masking(self):
        self.env.reset()
        action = {
            joint.name: {
                "position": 1.0,
                "velocity": 1e5,
                "torque": 1e5,
            }
            for joint in self.env.model.joints
        }
        self.env.step(action)
        # Commanded value in position
        self.assertAlmostEqual(
            self.env._spine.action["servo"]["left_hip"]["position"],
            1.0,
            places=5,
        )
        # Neutral action in other fields
        self.assertAlmostEqual(
            self.env._spine.action["servo"]["left_hip"]["velocity"],
            0.0,
            places=5,
        )
        self.assertAlmostEqual(
            self.env._spine.action["servo"]["left_hip"]["feedforward_torque"],
            0.0,
            places=5,
        )


if __name__ == "__main__":
    unittest.main()
