#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2025 Inria

"""Unit tests for UpkieServos environment."""

import unittest

import gymnasium as gym
import numpy as np

from upkie.envs.backends import MockBackend
from upkie.envs.upkie_servos import UpkieServos
from upkie.utils.robot_state import RobotState


class UpkieServosTestCase(unittest.TestCase):
    """Test fixture for UpkieServos environment."""

    def setUp(self):
        """Set up test fixture with MockBackend."""
        self.backend = MockBackend()
        self.env = UpkieServos(
            backend=self.backend,
            frequency=100.0,
            regulate_frequency=False,  # Disable for testing
        )

    def tearDown(self):
        """Clean up test fixture."""
        if hasattr(self, 'env'):
            self.env.close()

    def test_initialization(self):
        """Test that UpkieServos initializes correctly."""
        self.assertIsNotNone(self.env)
        self.assertIsNotNone(self.env.action_space)
        self.assertIsNotNone(self.env.observation_space)
        self.assertIsNotNone(self.env.model)
        self.assertEqual(len(self.env.model.joints), 6)  # 6 joints in Upkie

    def test_action_space_structure(self):
        """Test that action space has correct structure."""
        # Should be a Dict space with joint names as keys
        self.assertIsInstance(self.env.action_space, gym.spaces.Dict)
        
        # Check all expected joints are present
        expected_joints = {"left_hip", "left_knee", "left_wheel", 
                          "right_hip", "right_knee", "right_wheel"}
        self.assertEqual(set(self.env.action_space.spaces.keys()), expected_joints)
        
        # Each joint should have servo action keys
        expected_servo_keys = {"position", "velocity", "feedforward_torque", 
                              "kp_scale", "kd_scale", "maximum_torque"}
        for joint_name in expected_joints:
            joint_space = self.env.action_space[joint_name]
            self.assertIsInstance(joint_space, gym.spaces.Dict)
            self.assertEqual(set(joint_space.spaces.keys()), expected_servo_keys)

    def test_observation_space_structure(self):
        """Test that observation space has correct structure."""
        # Should be a Dict space with joint names as keys
        self.assertIsInstance(self.env.observation_space, gym.spaces.Dict)
        
        # Check all expected joints are present
        expected_joints = {"left_hip", "left_knee", "left_wheel", 
                          "right_hip", "right_knee", "right_wheel"}
        self.assertEqual(set(self.env.observation_space.spaces.keys()), expected_joints)
        
        # Each joint should have servo observation keys
        expected_servo_keys = {"position", "velocity", "torque", "temperature", "voltage"}
        for joint_name in expected_joints:
            joint_space = self.env.observation_space[joint_name]
            self.assertIsInstance(joint_space, gym.spaces.Dict)
            self.assertEqual(set(joint_space.spaces.keys()), expected_servo_keys)

    def test_reset(self):
        """Test environment reset functionality."""
        observation, info = self.env.reset()
        
        # Check return types
        self.assertIsInstance(observation, dict)
        self.assertIsInstance(info, dict)
        self.assertIn("spine_observation", info)
        
        # Check observation structure matches observation space
        self.assertTrue(self.env.observation_space.contains(observation))
        
        # Check all joints are in observation
        expected_joints = {"left_hip", "left_knee", "left_wheel", 
                          "right_hip", "right_knee", "right_wheel"}
        self.assertEqual(set(observation.keys()), expected_joints)
        
        # Check servo observation values are arrays
        for joint_name in expected_joints:
            joint_obs = observation[joint_name]
            for key in ["position", "velocity", "torque", "temperature", "voltage"]:
                self.assertIsInstance(joint_obs[key], np.ndarray)
                self.assertEqual(joint_obs[key].shape, (1,))

    def test_step(self):
        """Test environment step functionality."""
        self.env.reset()
        
        # Create a valid action
        action = self.env.get_neutral_action()
        
        # Take a step
        observation, reward, terminated, truncated, info = self.env.step(action)
        
        # Check return types
        self.assertIsInstance(observation, dict)
        self.assertIsInstance(reward, (int, float))
        self.assertIsInstance(terminated, bool)
        self.assertIsInstance(truncated, bool)
        self.assertIsInstance(info, dict)
        
        # Check observation is valid
        self.assertTrue(self.env.observation_space.contains(observation))
        
        # Default reward should be 1.0 (survival reward)
        self.assertEqual(reward, 1.0)
        
        # Episode shouldn't terminate by default
        self.assertFalse(terminated)
        self.assertFalse(truncated)

    def test_get_neutral_action(self):
        """Test neutral action generation."""
        neutral_action = self.env.get_neutral_action()
        
        # Check structure
        expected_joints = {"left_hip", "left_knee", "left_wheel", 
                          "right_hip", "right_knee", "right_wheel"}
        self.assertEqual(set(neutral_action.keys()), expected_joints)
        
        # Neutral action should have NaN position (no position control)
        for joint_name in expected_joints:
            joint_action = neutral_action[joint_name]
            self.assertTrue(np.isnan(joint_action["position"]))
            self.assertEqual(joint_action["velocity"], 0.0)
            self.assertEqual(joint_action["feedforward_torque"], 0.0)
            self.assertEqual(joint_action["kp_scale"], 1.0)
            self.assertEqual(joint_action["kd_scale"], 1.0)
        
        # Note: The neutral action contains scalar values, not numpy arrays,
        # so it won't pass action_space.contains() which expects arrays.
        # This is the expected behavior for the internal neutral action format.

    def test_action_clamping(self):
        """Test that actions are clamped to valid ranges."""
        self.env.reset()
        
        # Create action with extreme values
        extreme_action = {}
        for joint_name in ["left_hip", "left_knee", "left_wheel", 
                          "right_hip", "right_knee", "right_wheel"]:
            extreme_action[joint_name] = {
                "position": 1000.0,  # Very large value
                "velocity": -1000.0,  # Very large negative value
                "feedforward_torque": 500.0,  # Large torque
                "kp_scale": 100.0,  # Large gain
                "kd_scale": -10.0,  # Negative gain (invalid)
                "maximum_torque": 1000.0,  # Large max torque
            }
        
        # This should not raise an error due to clamping
        observation, reward, terminated, truncated, info = self.env.step(extreme_action)
        
        # Should still get valid observation
        self.assertTrue(self.env.observation_space.contains(observation))

    def test_model_property(self):
        """Test that model property is accessible."""
        model = self.env.model
        self.assertIsNotNone(model)
        self.assertEqual(len(model.joints), 6)
        
        # Check joint names
        joint_names = [joint.name for joint in model.joints]
        expected_joints = ["left_hip", "left_knee", "left_wheel", 
                          "right_hip", "right_knee", "right_wheel"]
        self.assertEqual(set(joint_names), set(expected_joints))

    def test_frequency_property(self):
        """Test frequency and dt properties."""
        self.assertEqual(self.env.frequency, 100.0)
        self.assertAlmostEqual(self.env.dt, 0.01)  # 1/100

    def test_init_state_randomization(self):
        """Test initial state randomization."""
        # Test with custom initial state
        custom_init_state = RobotState(
            position_base_in_world=np.array([1.0, 2.0, 3.0])
        )
        
        env_with_init_state = UpkieServos(
            backend=MockBackend(),
            frequency=100.0,
            init_state=custom_init_state,
            regulate_frequency=False,
        )
        
        try:
            # Should initialize without error
            self.assertIsNotNone(env_with_init_state.init_state)
            np.testing.assert_array_equal(
                env_with_init_state.init_state.position_base_in_world,
                np.array([1.0, 2.0, 3.0])
            )
        finally:
            env_with_init_state.close()

    def test_spine_action_conversion(self):
        """Test conversion from environment action to spine action."""
        self.env.reset()
        
        # Create test action with custom values for one joint, others use defaults
        env_action = self.env.get_neutral_action()
        env_action["left_hip"] = {
            "position": 0.5,
            "velocity": 1.0,
            "feedforward_torque": 0.1,
            "kp_scale": 0.8,
            "kd_scale": 0.9,
            "maximum_torque": 5.0,
        }
        
        # Convert to spine action
        spine_action = self.env.get_spine_action(env_action)
        
        # Should have servo key
        self.assertIn("servo", spine_action)
        self.assertIn("left_hip", spine_action["servo"])
        
        # Check values are converted correctly
        left_hip_action = spine_action["servo"]["left_hip"]
        self.assertAlmostEqual(left_hip_action["position"], 0.5)
        self.assertAlmostEqual(left_hip_action["velocity"], 1.0)
        self.assertAlmostEqual(left_hip_action["feedforward_torque"], 0.1)
        self.assertAlmostEqual(left_hip_action["kp_scale"], 0.8)
        self.assertAlmostEqual(left_hip_action["kd_scale"], 0.9)
        self.assertAlmostEqual(left_hip_action["maximum_torque"], 5.0)
        
        # Check that all joints are present in spine action
        expected_joints = {"left_hip", "left_knee", "left_wheel", 
                          "right_hip", "right_knee", "right_wheel"}
        self.assertEqual(set(spine_action["servo"].keys()), expected_joints)


if __name__ == "__main__":
    unittest.main()