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

"""Test clamping utility functions."""

import unittest

import numpy as np

from upkie.utils.clamp import clamp, clamp_abs, clamp_and_warn


class TestClamp(unittest.TestCase):
    def test_clamp(self):
        self.assertAlmostEqual(clamp(-2.1, -2.0, 2.0), -2.0)
        self.assertAlmostEqual(clamp(3.1, -2.0, 2.0), 2.0)

    def test_clamp_abs(self):
        self.assertAlmostEqual(clamp_abs(-2.1, 2.0), -2.0)
        self.assertAlmostEqual(clamp_abs(3.1, 2.0), 2.0)

    def test_clamp_and_warn(self):
        self.assertAlmostEqual(clamp_and_warn(-2.1, -2.0, 2.0, "x"), -2.0)
        self.assertAlmostEqual(clamp_and_warn(3.1, -2.0, 2.0, "x"), 2.0)

    def test_clamp_inf(self, x=1.42):
        self.assertAlmostEqual(clamp(x, -np.inf, np.inf), x)
        self.assertAlmostEqual(clamp_abs(x, np.inf), x)


if __name__ == "__main__":
    unittest.main()
