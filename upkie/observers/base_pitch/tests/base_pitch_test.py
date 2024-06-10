#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

"""
Test base pitch calculation.
"""

import unittest

import numpy as np

from upkie.observers.base_pitch import (
    compute_base_pitch_from_imu,
    compute_pitch_frame_in_parent,
)


class TestBasePitch(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()
