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
        self.tree = self.model.kinematic_tree

    def test_contact_frames_symmetric(self):
        """Left and right contact positions should be symmetric about y=0."""
        pos_left = self.tree.get_transform_frame_to_base(
            "left_wheel_tire"
        ).translation
        pos_right = self.tree.get_transform_frame_to_base(
            "right_wheel_tire"
        ).translation
        np.testing.assert_allclose(pos_left[0], pos_right[0], atol=1e-6)
        np.testing.assert_allclose(pos_left[1], -pos_right[1], atol=1e-6)
        np.testing.assert_allclose(pos_left[2], pos_right[2], atol=1e-6)

    def test_frame_names_contain_expected_links(self):
        names = self.tree.frame_names
        expected = [
            "base",
            "torso",
            "imu",
            "left_hip_qdd100_stator",
            "left_wheel_tire",
            "left_contact",
            "left_wheel_center",
            "right_hip_qdd100_stator",
            "right_wheel_tire",
            "right_contact",
            "right_wheel_center",
        ]
        for name in expected:
            self.assertIn(name, names)

    def test_joint_lists(self):
        for _ in range(3):  # do it thrice to make sure not a generator
            nb_joints = 0
            for joint in self.model.upper_leg_joints:
                nb_joints += 1
            for joint in self.model.wheel_joints:
                nb_joints += 1
            self.assertEqual(nb_joints, 6)

    def test_rotation_ars_to_world(self):
        """Check that the ARS orientation matches the pi3hat specification."""
        rotation_ars_to_world = np.diag([1.0, -1.0, -1.0])
        np.testing.assert_allclose(
            self.model.rotation_ars_to_world,
            rotation_ars_to_world,
            atol=1e-10,
        )

    def test_rotation_base_to_imu_from_urdf(self):
        """Check that rotation_base_to_imu is parsed from the URDF."""
        np.testing.assert_allclose(
            self.model.rotation_base_to_imu,
            np.diag([-1.0, 1.0, -1.0]),
            atol=1e-10,
        )

    def test_wheel_base(self):
        """Check that wheel_base is computed from the kinematic tree."""
        upkie_wheel_base = 0.3048  # meters
        self.assertAlmostEqual(
            self.model.wheel_base, upkie_wheel_base, places=2
        )

    def test_wheel_radius(self):
        """Check that wheel_radius is parsed from collision geometry."""
        upkie_wheel_radius = 0.05  # meters
        self.assertAlmostEqual(self.model.wheel_radius, upkie_wheel_radius)

    def test_wheel_tires_have_collision(self):
        for wheel_tire in ("left_wheel_tire", "right_wheel_tire"):
            link = self.tree.links[wheel_tire]
            self.assertTrue(len(link.collision_geometries) > 0)
            cylinder = link.collision_geometries[0]
            self.assertEqual(cylinder.shape, "cylinder")
            radius = cylinder.params["radius"]
            self.assertTrue(isinstance(radius, float))
            self.assertGreater(radius, 0.0)
            self.assertLess(radius, 1.0)


if __name__ == "__main__":
    unittest.main()
