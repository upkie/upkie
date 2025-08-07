#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

"""Test upkie.envs submodule."""

import unittest

import gymnasium as gym


class UpkieEnvsTestCase(unittest.TestCase):
    def test_unregistered(self):
        with self.assertRaises(gym.error.NameNotFound):
            gym.make("Upkie-Servos-NotFound")
        with gym.make("Upkie-Mock-Servos") as env:
            self.assertIsNotNone(env)


if __name__ == "__main__":
    unittest.main()
