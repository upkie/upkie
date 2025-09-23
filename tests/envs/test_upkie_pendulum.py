#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

"""Test UpkiePendulum wrapper."""

import unittest

import gymnasium as gym
import numpy as np
from upkie.config import ROBOT_CONFIG
from upkie.envs.backends import MockBackend
from upkie.envs.upkie_pendulum import UpkiePendulum
from upkie.envs.upkie_servos import UpkieServos
from upkie.exceptions import UpkieException


class PendulumTestCase(unittest.TestCase):
    def setUp(self):
        self.backend = MockBackend()
        servos_env = UpkieServos(
            backend=self.backend,
            frequency=100.0,
        )
        pendulum = UpkiePendulum(
            servos_env,
            fall_pitch=1.0,
            max_ground_velocity=1.0,
        )
        self.servos_env = servos_env
        self.env = pendulum

    def test_reset(self):
        observation, info = self.env.reset()
        spine_observation = info["spine_observation"]
        self.assertAlmostEqual(
            observation[1], spine_observation["wheel_odometry"]["position"]
        )
        self.assertGreaterEqual(spine_observation["number"], 1)

    def test_reward(self):
        observation, _ = self.env.reset()
        action = np.zeros(self.env.action_space.shape)
        observation, reward, terminated, truncated, _ = self.env.step(action)
        self.assertNotEqual(reward, 0.0)  # non-zero base velocity

        spine_observation = self.backend.get_spine_observation()
        base_orientation = spine_observation["base_orientation"]
        base_orientation["pitch"] = 0.0
        base_orientation["angular_velocity"] = [0.0, 0.0, 0.0]
        base_orientation["linear_velocity"] = [0.0, 0.0, 0.0]
        observation, reward, terminated, truncated, _ = self.env.step(action)
        self.assertAlmostEqual(reward, 1.0)  # ideal base state

    def test_check_env(self):
        try:
            from stable_baselines3.common.env_checker import check_env

            check_env(self.env)
        except ImportError:
            pass

    def test_maximum_torques(self):
        _, _ = self.env.reset()
        action = np.zeros(self.env.action_space.shape)
        _, _, _, _, _ = self.env.step(action)
        spine_action = self.backend.get_last_action()
        servo_action = spine_action["servo"]
        model = self.env.get_wrapper_attr("model")
        for joint in model.upper_leg_joints:
            maximum_torque = servo_action[joint.name]["maximum_torque"]
            self.assertLess(maximum_torque, 20.0)
        for wheel in model.wheel_joints:
            maximum_torque = servo_action[wheel.name]["maximum_torque"]
            self.assertLess(maximum_torque, 2.0)

    def test_max_ground_velocity(self):
        _, _ = self.env.reset()
        action = np.full(self.env.action_space.shape, 1111.1)
        _, _, _, _, _ = self.env.step(action)
        spine_action = self.backend.get_last_action()
        servo_action = spine_action["servo"]
        max_ground_velocity = self.env.action_space.high[0]
        model = self.env.get_wrapper_attr("model")
        for wheel in model.wheel_joints:
            wheel_velocity = servo_action[wheel.name]["velocity"]  # rad/s
            wheel_radius = ROBOT_CONFIG["wheel_radius"]
            ground_velocity = np.abs(wheel_velocity * wheel_radius)
            self.assertLess(ground_velocity, max_ground_velocity + 1e-10)

    def test_dtype_consistency(self):
        """Test that action and observation spaces use float32 dtype."""
        self.assertEqual(self.env.action_space.dtype, np.float32)
        self.assertEqual(self.env.observation_space.dtype, np.float32)

        # Check that observations from reset are float32
        observation, _ = self.env.reset()
        self.assertEqual(observation.dtype, np.float32)

        # Check that observations from step are float32
        action = np.zeros(self.env.action_space.shape, dtype=np.float32)
        observation, _, _, _, _ = self.env.step(action)
        self.assertEqual(observation.dtype, np.float32)

    def test_frequency_validation_missing_attribute(self):
        """An exception is raised when the environment has no frequency."""

        class MockEnvWithoutFrequency(gym.Env):
            """Mock environment that doesn't have frequency attribute."""

            def __init__(self):
                # Don't set frequency attribute
                pass

            def step(self, action):
                return None, 0.0, False, False, {}

            def reset(self, **kwargs):
                return None, {}

        mock_env = MockEnvWithoutFrequency()

        with self.assertRaises(UpkieException) as context:
            UpkiePendulum(mock_env)

        self.assertIn("frequency", str(context.exception))

    def test_frequency_validation_none_value(self):
        """An exception is raised when environment has a None frequency."""

        class MockEnvWithNoneFrequency(gym.Env):
            """Mock environment with frequency set to None."""

            def __init__(self):
                self.frequency = None

            def step(self, action):
                return None, 0.0, False, False, {}

            def reset(self, **kwargs):
                return None, {}

        mock_env = MockEnvWithNoneFrequency()

        with self.assertRaises(UpkieException) as context:
            UpkiePendulum(mock_env)

        self.assertIn("frequency", str(context.exception))


if __name__ == "__main__":
    unittest.main()
