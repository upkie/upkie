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
Test forward and inverse kinematics functions.
"""

import unittest

from upkie_locomotion.agents.blue_balancer.kinematics import (
    forward_kinematics,
    inverse_kinematics,
)


class TestKinematics(unittest.TestCase):
    def test_inverse_maps(self):
        """
        Check that IK is the inverse of FK.
        """
        crouch_height = 0.1
        limb_length = 0.3
        (q_hip, q_knee) = inverse_kinematics(crouch_height, limb_length)
        self.assertAlmostEqual(
            forward_kinematics(q_hip, q_knee, limb_length), crouch_height
        )


if __name__ == "__main__":
    unittest.main()
