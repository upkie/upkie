#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

"""Test Model class."""

import unittest

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

    def test_rotation_base_to_imu_from_urdf(self):
        """Check that rotation_base_to_imu is parsed from the URDF."""
        np.testing.assert_allclose(
            self.model.rotation_base_to_imu,
            np.diag([-1.0, 1.0, -1.0]),
            atol=1e-10,
        )


if __name__ == "__main__":
    unittest.main()
