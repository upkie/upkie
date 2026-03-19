#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

"""Test upkie.envs submodule."""

import unittest

import gymnasium as gym


class UpkieEnvsTestCase(unittest.TestCase):
    def test_unregistered(self):
        with self.assertRaises(gym.error.NameNotFound):
            gym.make("Upkie-Servos-NotFound")
        with gym.make("Upkie-Mock-Servos") as env:
            self.assertIsNotNone(env)

    def test_cross_button_terminates(self):
        """Joystick A/cross button press sets both terminated and truncated."""
        with gym.make("Upkie-Mock-Servos") as env:
            upkie_env = env.unwrapped
            original_step = upkie_env.backend.step

            def step_with_cross_button(action):
                obs = original_step(action)
                obs["joystick"] = {"cross_button": 1}
                return obs

            upkie_env.backend.step = step_with_cross_button
            env.reset()
            action = upkie_env.get_neutral_action()
            _, _, terminated, truncated, _ = env.step(action)
            self.assertTrue(terminated)
            self.assertTrue(truncated)

    def test_log(self):
        """Test that logged entries are sent to backend in spine action."""
        with gym.make("Upkie-Mock-Servos") as env:
            upkie_env = env.unwrapped

            # Mock the backend to capture the action sent to it
            original_step = upkie_env.backend.step
            captured_action = None

            def mock_step(action):
                nonlocal captured_action
                captured_action = action
                return original_step(action)

            upkie_env.backend.step = mock_step

            # Reset environment
            observation, info = env.reset()

            # Log some entries
            upkie_env.log("test_key", "test_value")
            upkie_env.log("number", 123)

            # Take a step to trigger sending the logged data
            action = upkie_env.get_neutral_action()
            observation, reward, terminated, truncated, info = env.step(action)

            # Verify the action contains the "env" key with our logged data
            self.assertIsNotNone(captured_action)
            self.assertIn("env", captured_action)
            env_data = captured_action["env"]
            self.assertEqual(env_data["test_key"], "test_value")
            self.assertEqual(env_data["number"], 123)


if __name__ == "__main__":
    unittest.main()
