#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

"""Tests for the PointContact data class."""

import unittest

import numpy as np

from upkie.utils.point_contact import PointContact


class PointContactTestCase(unittest.TestCase):
    """Test cases for PointContact data class."""

    def test_point_contact_creation(self):
        """Test basic PointContact creation with direct constructor."""
        point_contact = PointContact(
            link_name="left_wheel_link",
            position_contact_in_world=np.array([0.1, 0.2, 0.3]),
            force_in_world=np.array([10.0, 20.0, 30.0]),
        )

        self.assertEqual(point_contact.link_name, "left_wheel_link")
        np.testing.assert_array_equal(
            point_contact.position_contact_in_world, [0.1, 0.2, 0.3]
        )
        np.testing.assert_array_equal(
            point_contact.force_in_world, [10.0, 20.0, 30.0]
        )

    def test_data_class_representation(self):
        """Test PointContact string representation."""
        point_contact = PointContact(
            link_name="imu",
            position_contact_in_world=np.array([0.0, 0.0, 0.1]),
            force_in_world=np.array([0.0, 0.0, -50.0]),
        )

        repr_str = repr(point_contact)
        self.assertIn("PointContact", repr_str)
        self.assertIn("link_name='imu'", repr_str)
        self.assertIn("array", repr_str)


if __name__ == "__main__":
    unittest.main()
