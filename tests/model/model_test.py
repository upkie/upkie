#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

"""Test Model class."""

import unittest

import upkie_description

from upkie.model import Model


class TestModel(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()
