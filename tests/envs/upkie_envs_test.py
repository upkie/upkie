#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

"""Test upkie.envs submodule."""

import unittest

import gymnasium as gym

import upkie.envs


class UpkieEnvsTestCase(unittest.TestCase):
    def test_unregistered(self):
        with self.assertRaises(gym.error.NameNotFound):
            gym.make("Upkie-Servos-NotFound")
        with self.assertRaises(gym.error.NameNotFound):
            gym.make("Upkie-Mock-Servos")  # should not work yet
        upkie.envs.register()
        with gym.make("Upkie-Mock-Servos") as env:
            self.assertIsNotNone(env)  # should work now


if __name__ == "__main__":
    unittest.main()
