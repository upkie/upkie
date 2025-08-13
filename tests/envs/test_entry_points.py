#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

"""Test Gymnasium environment entry points."""

import unittest
from multiprocessing.shared_memory import SharedMemory

import gymnasium as gym

from upkie.exceptions import UpkieTimeoutError


class EntryPointsTestCase(unittest.TestCase):
    def test_mock_servos(self):
        with gym.make("Upkie-Mock-Servos") as env:
            self.assertIsNotNone(env)

    def test_mock_pendulum(self):
        with gym.make("Upkie-Mock-Pendulum") as env:
            self.assertIsNotNone(env)

    def test_spine_servos(self):
        shm = SharedMemory(name=None, size=42, create=True)
        try:
            env = gym.make("Upkie-Spine-Servos", shm_name=shm._name)
            self.assertIsNotNone(env)
            try:
                del env  # we delete it explicitly
            except UpkieTimeoutError:  # to catch this exception
                pass  # which is ok: there is no spine, thus no response
        finally:
            shm.close()

    def test_spine_pendulum(self):
        shm = SharedMemory(name=None, size=42, create=True)
        try:
            env = gym.make("Upkie-Spine-Pendulum", shm_name=shm._name)
            self.assertIsNotNone(env)
            try:
                del env  # we delete it explicitly
            except UpkieTimeoutError:  # to catch this exception
                pass  # which is ok: there is no spine, thus no response
        finally:
            shm.close()

    def test_pybullet_servos(self):
        with gym.make("Upkie-PyBullet-Servos", gui=False) as env:
            self.assertIsNotNone(env)

    def test_pybullet_pendulum(self):
        with gym.make("Upkie-PyBullet-Pendulum", gui=False) as env:
            self.assertIsNotNone(env)

    def test_unregistered(self):
        with self.assertRaises(gym.error.NameNotFound):
            gym.make("Upkie-Servos-NotFound")
        with gym.make("Upkie-Mock-Servos") as env:
            self.assertIsNotNone(env)


if __name__ == "__main__":
    unittest.main()
