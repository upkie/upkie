#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Upkie Team

"""Test UpkieNavigation wrapper."""

import unittest

import numpy as np

from upkie.envs.backends import MockBackend
from upkie.envs.upkie_navigation import UpkieNavigation
from upkie.envs.upkie_pendulum import UpkiePendulum
from upkie.envs.upkie_servos import UpkieServos


class NavigationTestCase(unittest.TestCase):
    def setUp(self):
        """Set up test environment with mock backend."""
        self.backend = MockBackend()
        servos_env = UpkieServos(
            backend=self.backend,
            frequency=100.0,
        )
        pendulum_env = UpkiePendulum(
            servos_env,
            fall_pitch=1.0,
            max_ground_velocity=1.0,
        )
        navigation_env = UpkieNavigation(
            pendulum_env,
            max_linear_velocity=1.0,
            max_angular_velocity=2.0,
        )
        self.servos_env = servos_env
        self.pendulum_env = pendulum_env
        self.env = navigation_env

    def test_action_space_dtype(self):
        """Test that action space has correct dtype np.float32."""
        self.assertEqual(self.env.action_space.dtype, np.float32)
        self.assertEqual(self.env.action_space.shape, (2,))
        # Test limits
        np.testing.assert_array_equal(
            self.env.action_space.low, np.array([-1.0, -2.0], dtype=np.float32)
        )
        np.testing.assert_array_equal(
            self.env.action_space.high, np.array([1.0, 2.0], dtype=np.float32)
        )

    def test_observation_space_dtype(self):
        """Test that observation space has correct dtype."""
        # UpkieNavigation observation space should be float (not float32)
        self.assertEqual(self.env.observation_space.dtype, float)
        self.assertEqual(self.env.observation_space.shape, (3,))
        # Check SE(2) limits: [x, y, theta]
        expected_low = np.array([-np.inf, -np.inf, -np.pi], dtype=float)
        expected_high = np.array([np.inf, np.inf, np.pi], dtype=float)
        np.testing.assert_array_equal(
            self.env.observation_space.low, expected_low
        )
        np.testing.assert_array_equal(
            self.env.observation_space.high, expected_high
        )

    def test_reset(self):
        """Test reset functionality."""
        observation, info = self.env.reset()

        # Check observation shape and type
        self.assertIsInstance(observation, np.ndarray)
        self.assertEqual(observation.shape, (3,))
        self.assertEqual(observation.dtype, float)

        # Check SE(2) pose initialization
        np.testing.assert_array_equal(observation, np.array([0.0, 0.0, 0.0]))

        # Check info contains spine observation
        self.assertIn("spine_observation", info)
        spine_observation = info["spine_observation"]
        self.assertIn("wheel_odometry", spine_observation)

    def test_step_with_zero_action(self):
        """Test step with zero navigation action."""
        observation, _ = self.env.reset()

        # Zero action: no linear or angular velocity
        action = np.array([0.0, 0.0], dtype=np.float32)

        next_observation, reward, terminated, truncated, info = self.env.step(
            action
        )

        # Check return types
        self.assertIsInstance(next_observation, np.ndarray)
        self.assertEqual(next_observation.shape, (3,))
        self.assertEqual(next_observation.dtype, float)
        self.assertIsInstance(reward, (int, float))
        self.assertIsInstance(terminated, bool)
        self.assertIsInstance(truncated, bool)
        self.assertIsInstance(info, dict)

    def test_step_with_linear_action(self):
        """Test step with linear velocity action."""
        observation, _ = self.env.reset()
        initial_pose = observation.copy()

        # Forward motion: positive linear velocity, zero angular
        action = np.array([0.1, 0.0], dtype=np.float32)
        next_observation, _, _, _, _ = self.env.step(action)

        # Check that pose has been updated (should move forward in x)
        self.assertNotEqual(next_observation[0], initial_pose[0])  # x changed
        self.assertEqual(next_observation[1], initial_pose[1])  # y unchanged
        self.assertEqual(
            next_observation[2], initial_pose[2]
        )  # theta unchanged

    def test_step_with_angular_action(self):
        """Test step with angular velocity action."""
        observation, _ = self.env.reset()
        initial_pose = observation.copy()

        # Pure rotation: zero linear velocity, positive angular
        action = np.array([0.0, 0.5], dtype=np.float32)
        next_observation, _, _, _, _ = self.env.step(action)

        # Check that yaw angle has changed and translation hasn't moved much
        self.assertAlmostEqual(next_observation[0], initial_pose[0], places=4)
        self.assertAlmostEqual(next_observation[1], initial_pose[1], places=4)
        self.assertNotEqual(next_observation[2], initial_pose[2])

    def test_se2_pose_tracking(self):
        """Test SE(2) pose tracking over multiple steps."""
        observation, _ = self.env.reset()

        # Move forward and turn
        forward_action = np.array([0.1, 0.0], dtype=np.float32)
        turn_action = np.array([0.0, 0.1], dtype=np.float32)

        poses = [observation.copy()]

        # Take several steps with different actions
        for action in [
            forward_action,
            forward_action,
            turn_action,
            forward_action,
        ]:
            observation, _, _, _, _ = self.env.step(action)
            poses.append(observation.copy())

        # Check that poses are different and track SE(2) motion
        for i in range(1, len(poses)):
            # Each pose should be different from the previous one
            self.assertFalse(np.array_equal(poses[i], poses[i - 1]))

        # Check that all poses are valid SE(2) poses
        for pose in poses:
            self.assertEqual(len(pose), 3)  # [x, y, theta]
            # Theta should be normalized to [-pi, pi]
            self.assertGreaterEqual(pose[2], -np.pi)
            self.assertLessEqual(pose[2], np.pi)

    def test_action_clamping(self):
        """Test that actions are clamped to action space bounds."""
        observation, _ = self.env.reset()

        # Action beyond bounds
        extreme_action = np.array([10.0, -10.0], dtype=np.float32)

        # Should not raise error due to clamping
        next_observation, _, _, _, _ = self.env.step(extreme_action)
        self.assertIsInstance(next_observation, np.ndarray)
        self.assertEqual(next_observation.shape, (3,))

    def test_mpc_balancer_integration(self):
        """Test that MPC balancer is properly integrated."""
        # Check that MPC balancer is initialized
        self.assertIsNotNone(self.env.mpc_balancer)

        # Test that it has expected attributes
        self.assertTrue(
            hasattr(self.env.mpc_balancer, "compute_ground_velocity")
        )

        # Test step uses MPC balancer
        observation, _ = self.env.reset()
        action = np.array([0.1, 0.0], dtype=np.float32)

        # Should complete without error (MPC balancer working)
        next_observation, _, _, _, _ = self.env.step(action)
        self.assertIsInstance(next_observation, np.ndarray)

    def test_check_env(self):
        """Test Stable Baselines3 environment checker if available."""
        try:
            from stable_baselines3.common.env_checker import check_env

            check_env(self.env)
        except ImportError:
            # Skip test if stable_baselines3 not available
            pass

    def test_wrapped_environment_access(self):
        """Test that we can access the wrapped UpkiePendulum environment."""
        self.assertIsInstance(self.env.env, UpkiePendulum)
        self.assertIsInstance(self.env.env.env, UpkieServos)

        # Test that we can access dt through the wrapped environment
        self.assertIsNotNone(self.env.env.unwrapped.dt)
        self.assertGreater(self.env.env.unwrapped.dt, 0)


if __name__ == "__main__":
    unittest.main()
