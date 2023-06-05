#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2022 Stéphane Caron
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Test base pitch calculation.
"""

import unittest

import numpy as np

from observers.base_pitch import (
    compute_base_pitch_from_imu,
    compute_pitch_frame_in_parent,
)


class TestBasePitch(unittest.TestCase):
    def test_zero_pitch(self, phi=0.42):
        """
        Check that a pure rotation around the z-axis yields no pitch.
        """
        orientation_imu_in_world = np.array(
            [
                [np.cos(phi), -np.sin(phi), 0.0],
                [np.sin(phi), np.cos(phi), 0.0],
                [0.0, 0.0, 1.0],
            ]
        )
        imu_pitch = compute_pitch_frame_in_parent(orientation_imu_in_world)
        self.assertEqual(imu_pitch, 0.0)

    def test_close_to_zero(self, theta=1e-3):
        """
        Check that a pure rotation around the y-axis yields the correct pitch.
        """
        orientation_imu_in_world = np.array(
            [
                [np.cos(theta), 0.0, np.sin(theta)],
                [0.0, 1.0, 0.0],
                [-np.sin(theta), 0.0, np.cos(theta)],
            ]
        )
        imu_pitch = compute_pitch_frame_in_parent(orientation_imu_in_world)
        self.assertTrue(np.isclose(imu_pitch, theta))

    def test_orientation_not_neatly_normalized(self, theta=1e-3):
        """
        As an extra, the function can handle a partially-normalized input. We
        are going the extra mile here, this is not strictly necessary.
        """
        orientation_imu_in_world = np.array(
            [
                [np.cos(theta), 0.0, np.sin(theta)],
                [0.0, 1.0, 0.0],
                [-np.sin(theta), 0.0, np.cos(theta)],
            ]
        )
        orientation_imu_in_world[:, 0] *= 1.0 - 1e-2
        imu_pitch = compute_pitch_frame_in_parent(orientation_imu_in_world)
        self.assertTrue(np.isclose(imu_pitch, theta))

    def test_balancing_plane_lateral_in_imu_frame(self):
        quat_imu_in_ars = np.array(
            [
                0.008472769239730098,
                -0.9953038144146671,
                -0.09639792825405252,
                -0.002443076206500708,
            ]
        )
        rotation_imu_to_ars = np.array(
            [
                [0.98140294, 0.19193185, 0.00322969],
                [0.19184905, -0.9812713, 0.01733697],
                [0.00649672, -0.01639494, -0.99984449],
            ]
        )
        pitch_imu_in_ars = compute_pitch_frame_in_parent(
            rotation_imu_to_ars, sagittal_axis=1, vertical_axis=2
        )
        self.assertAlmostEqual(pitch_imu_in_ars, 3.125, places=3)
        pitch_imu_in_world = compute_base_pitch_from_imu(
            quat_imu_in_ars, sagittal_axis=1, vertical_axis=2
        )
        self.assertAlmostEqual(pitch_imu_in_world, -0.016, places=3)


if __name__ == "__main__":
    unittest.main()
