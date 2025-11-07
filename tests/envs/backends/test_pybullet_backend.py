#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2025 Inria

"""Tests for PyBullet backend functionality with a mock pybullet module."""

import unittest

import numpy as np
from scipy.spatial.transform import Rotation as ScipyRotation
from upkie.envs.backends.pybullet_backend import PyBulletBackend
from upkie.utils.robot_state import RobotState


class PyBulletBackendTestCase(unittest.TestCase):
    def setUp(self):
        self.backend = PyBulletBackend(dt=5e-3, gui=False)
        self.backend.reset(
            init_state=RobotState(
                position_base_in_world=np.array([0.0, 0.0, 0.6])
            )
        )

    def tearDown(self):
        self.backend.close()

    def test_step_return_type(self):
        """Check that step takes an action dict and returns a dict as well."""
        spine_observation = self.backend.step(action={})
        self.assertIsInstance(spine_observation, dict)

    def test_initial_pitch(self):
        """Check that step takes an action dict and returns a dict as well."""
        spine_observation = self.backend.step(action={})
        reset_pitch = spine_observation["base_orientation"]["pitch"]
        self.assertAlmostEqual(reset_pitch, 0.0)

    def test_fall_pitch(self):
        """Pitch goes above a threshold after several steps with no action."""
        for _ in range(100):
            spine_observation = self.backend.step(action={})
        fall_pitch = spine_observation["base_orientation"]["pitch"]
        self.assertGreater(abs(fall_pitch), 0.5)

    def test_fall_pitch_with_custom_initial_yaw(self):
        """Check that robot also falls from a custom initial state.

        See https://github.com/upkie/upkie/issues/527 for context.
        """
        yaw_pitch_roll = np.array([np.pi / 2, 0.0, 0.0])
        init_orientation = ScipyRotation.from_euler("ZYX", yaw_pitch_roll)
        self.backend._reset_robot_state(
            RobotState(orientation_base_in_world=init_orientation)
        )
        for _ in range(100):
            spine_observation = self.backend.step(action={})
        fall_pitch = spine_observation["base_orientation"]["pitch"]
        self.assertGreater(abs(fall_pitch), 0.5)
