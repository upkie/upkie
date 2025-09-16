#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

"""Test clamping utility functions."""

import unittest

import numpy as np

from upkie.utils.clamp import clamp, clamp_abs, clamp_and_warn


class ClampTestCase(unittest.TestCase):
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
