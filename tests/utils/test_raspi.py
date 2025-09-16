#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

"""Test Raspberry Pi utility functions."""

import unittest

from upkie.utils.raspi import on_raspi


class RaspiTestCase(unittest.TestCase):
    def test_on_raspi(self):
        self.assertFalse(on_raspi())


if __name__ == "__main__":
    unittest.main()
