#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

"""Test KinematicTree class."""

import unittest

import numpy as np
import upkie_description

from upkie.exceptions import ModelError
from upkie.model.kinematic_tree import KinematicTree
from upkie.model.se3 import SE3


class KinematicTreeTestCase(unittest.TestCase):
    def setUp(self):
        self.tree = KinematicTree(upkie_description.URDF_PATH)

    def test_base_to_base_is_identity(self):
        transform = self.tree.get_transform_frame_to_base("base")
        self.assertTrue(transform.is_approx(SE3.identity()))

    def test_root_is_base(self):
        """Check that the base frame is called 'base'."""
        self.assertEqual(self.tree._root, "base")

    def test_rotation_base_to_imu(self):
        """Check that the tree has an IMU frame."""
        self.assertTrue("imu" in self.tree.frame_names)

    def test_torso_transform(self):
        transform = self.tree.get_transform_frame_to_base("torso")
        np.testing.assert_allclose(
            transform.translation, [0.0, 0.0, -0.1], atol=1e-10
        )
        np.testing.assert_allclose(transform.rotation, np.eye(3), atol=1e-10)

    def test_transform_roundtrip(self):
        """get_transform(A, B) * get_transform(B, A) should be identity."""
        pairs = [
            ("base", "torso"),
            ("base", "imu"),
            ("torso", "left_hip_qdd100_stator"),
            ("left_wheel_tire", "left_contact"),
        ]
        for a, b in pairs:
            with self.subTest(a=a, b=b):
                t_ab = self.tree.get_transform(a, b)
                t_ba = self.tree.get_transform(b, a)
                product = t_ab * t_ba
                self.assertTrue(
                    product.is_approx(SE3.identity()),
                    f"get_transform({a},{b}) * get_transform({b},{a}) "
                    f"is not identity",
                )

    def test_unknown_frame_raises(self):
        with self.assertRaises(ModelError):
            self.tree.get_transform_frame_to_base("nonexistent")
        with self.assertRaises(ModelError):
            self.tree.get_transform("base", "nonexistent")


if __name__ == "__main__":
    unittest.main()
