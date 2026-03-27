#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

"""Test rotation conversion utilities."""

import unittest

import numpy as np

from upkie.utils.rotations import rotation_matrix_from_rpy


class RotationMatrixFromRpyTestCase(unittest.TestCase):
    def test_identity(self):
        """Zero RPY yields the identity matrix."""
        np.testing.assert_allclose(
            rotation_matrix_from_rpy((0.0, 0.0, 0.0)),
            np.eye(3),
            atol=1e-10,
        )

    def test_roll_pi(self):
        """Pure roll of pi rotates y and z axes."""
        expected = np.diag([1.0, -1.0, -1.0])
        np.testing.assert_allclose(
            rotation_matrix_from_rpy((np.pi, 0.0, 0.0)),
            expected,
            atol=1e-10,
        )

    def test_pitch_pi(self):
        """Pure pitch of pi rotates x and z axes."""
        expected = np.diag([-1.0, 1.0, -1.0])
        np.testing.assert_allclose(
            rotation_matrix_from_rpy((0.0, np.pi, 0.0)),
            expected,
            atol=1e-10,
        )

    def test_yaw_pi(self):
        """Pure yaw of pi rotates x and y axes."""
        expected = np.diag([-1.0, -1.0, 1.0])
        np.testing.assert_allclose(
            rotation_matrix_from_rpy((0.0, 0.0, np.pi)),
            expected,
            atol=1e-10,
        )

    def test_roll_pi_yaw_pi(self):
        """RPY (pi, 0, pi) as used in Upkie's IMU placement."""
        expected = np.diag([-1.0, 1.0, -1.0])
        np.testing.assert_allclose(
            rotation_matrix_from_rpy((np.pi, 0.0, np.pi)),
            expected,
            atol=1e-10,
        )

    def test_yaw_half_pi(self):
        """Yaw of pi/2 rotates x to y."""
        expected = np.array([
            [0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        ])
        np.testing.assert_allclose(
            rotation_matrix_from_rpy((0.0, 0.0, np.pi / 2)),
            expected,
            atol=1e-10,
        )

    def test_result_is_proper_rotation(self):
        """Result is orthogonal with determinant +1."""
        rpy = (0.3, -0.7, 1.2)
        R = rotation_matrix_from_rpy(rpy)
        np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-10)
        self.assertAlmostEqual(np.linalg.det(R), 1.0)


if __name__ == "__main__":
    unittest.main()
