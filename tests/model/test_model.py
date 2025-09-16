#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

"""Test Model class."""

import unittest
from unittest.mock import patch

import numpy as np
import upkie_description

from upkie.model import Model


class ModelTestCase(unittest.TestCase):
    def setUp(self):
        self.model = Model(upkie_description.URDF_PATH)

    def test_joint_lists(self):
        for _ in range(3):  # do it thrice to make sure not a generator
            nb_joints = 0
            for joint in self.model.upper_leg_joints:
                nb_joints += 1
            for joint in self.model.wheel_joints:
                nb_joints += 1
            self.assertEqual(nb_joints, 6)

    def test_default_rotation_base_to_imu(self):
        """Load the default `rotation_base_to_imu` from the robot config."""
        # Test default value
        expected_rotation = np.diag([-1.0, 1.0, -1.0])
        np.testing.assert_array_equal(
            self.model.rotation_base_to_imu, expected_rotation
        )

    @patch("upkie.model.model.ROBOT_CONFIG")
    def test_custom_rotation_base_to_imu(self, mock_robot_config):
        """Load a custom `rotation_base_to_imu` from a user robot config."""
        custom_rotation = np.diag([1.0, -1.0, 1.0])
        mock_robot_config.__getitem__.return_value = custom_rotation
        model = Model()
        np.testing.assert_array_equal(
            model.rotation_base_to_imu, custom_rotation
        )


if __name__ == "__main__":
    unittest.main()
