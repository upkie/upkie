# SPDX-License-Identifier: Apache-2.0

"""Test Model class."""

import unittest

import cookie_description
import numpy as np
import upkie_description

from upkie.exceptions import ModelError
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

    def test_upkie_is_left_wheeled(self):
        """Upkie robots are left-wheeled."""
        self.assertTrue(self.model.left_wheeled)

    def test_cookie_is_right_wheeled(self):
        """Cookie robots are right-wheeled."""
        cookie_model = Model(cookie_description.URDF_PATH)
        self.assertFalse(cookie_model.left_wheeled)

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

    def test_leg_segment_length(self):
        """Segment lengths are half the hip-to-axle distance."""
        self.assertAlmostEqual(self.model.leg_segment_length, 0.1715)
        cookie_model = Model(cookie_description.URDF_PATH)
        self.assertAlmostEqual(cookie_model.leg_segment_length, 0.227)

    def test_zero_crouching_height_is_straight(self):
        angles = self.model.compute_crouching_posture(0.0)
        for joint in self.model.upper_leg_joints:
            self.assertEqual(angles[joint.name], 0.0)

    def test_crouching_posture(self):
        """Symmetric legs shorten by the crouching height, knees forward."""
        ell = self.model.leg_segment_length
        for height in (0.01, 0.05, 0.1, 0.2, 0.3):
            angles = self.model.compute_crouching_posture(height)
            hip, knee = angles["left_hip"], angles["left_knee"]
            self.assertGreater(hip, 0.0)
            self.assertAlmostEqual(knee, -2.0 * hip)
            self.assertAlmostEqual(2.0 * ell * np.cos(hip), 2.0 * ell - height)
            self.assertEqual(angles["right_hip"], -hip)
            self.assertEqual(angles["right_knee"], -knee)

    def test_crouching_out_of_reach(self):
        ell = self.model.leg_segment_length
        with self.assertRaises(ModelError):
            self.model.compute_crouching_posture(-0.01)
        with self.assertRaises(ModelError):
            self.model.compute_crouching_posture(2.0 * ell)


if __name__ == "__main__":
    unittest.main()
