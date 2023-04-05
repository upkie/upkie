#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria
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

"""Test Pinocchio utility functions."""

import unittest

import numpy as np
import pinocchio as pin

from upkie_locomotion.utils.pinocchio import (
    box_position_limits,
    box_torque_limits,
    box_velocity_limits,
)


class TestPinocchio(unittest.TestCase):
    def test_box_limits(self):
        model = pin.Model()
        model.addJoint(
            0,
            pin.JointModelRevoluteUnboundedUnaligned(),
            pin.SE3.Identity(),
            "J1",
        )
        model.addJoint(
            1,
            pin.JointModelRevoluteUnaligned(),
            pin.SE3.Identity(),
            "revolute",
            max_effort=np.array([1.0]),
            max_velocity=np.array([42.0]),
            min_config=np.array([-12.0]),
            max_config=np.array([12.0]),
        )
        model.addJoint(
            2,
            pin.JointModelRevoluteUnaligned(),
            pin.SE3.Identity(),
            "revolute",
            max_effort=np.array([0.0]),
            max_velocity=np.array([0.0]),
            min_config=np.array([0.0]),
            max_config=np.array([0.0]),
        )
        q_min, q_max = box_position_limits(model)
        v_max = box_velocity_limits(model)
        tau_max = box_torque_limits(model)
        print(f"{q_max=}")
        self.assertTrue(np.allclose(q_max, [+np.inf, +np.inf, 12.0, +np.inf]))
        self.assertTrue(np.allclose(q_min, [-np.inf, -np.inf, -12.0, -np.inf]))
        self.assertTrue(np.allclose(v_max, [+np.inf, 42.0, +np.inf]))
        self.assertTrue(np.allclose(tau_max, [+np.inf, 1.0, +np.inf]))


if __name__ == "__main__":
    unittest.main()
