#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

"""Test UpkieMockEnv environment."""

import unittest

import gymnasium as gym

import upkie.envs


class UpkieMockEnvTestCase(unittest.TestCase):
    def test_registration(self):
        upkie.envs.register()
        with gym.make("Upkie-Mock-Servos") as env:
            self.assertIsNotNone(env)


if __name__ == "__main__":
    unittest.main()
