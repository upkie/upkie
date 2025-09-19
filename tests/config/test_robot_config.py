#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2025 Inria

import unittest

import numpy as np

from upkie.config.robot_config import (
    _DEFAULT_ROBOT_CONFIG,
    ROBOT_CONFIG,
    _merge_user_robot_config,
)


class RobotConfigTestCase(unittest.TestCase):
    """!
    Test robot configuration functionality.
    """

    def test_default_robot_config_structure(self):
        """Test that default robot config has expected structure."""
        self.assertIn("leg_length", _DEFAULT_ROBOT_CONFIG)
        self.assertIn("wheel_radius", _DEFAULT_ROBOT_CONFIG)
        self.assertIn("mass", _DEFAULT_ROBOT_CONFIG)
        self.assertIn("rotation_base_to_imu", _DEFAULT_ROBOT_CONFIG)

    def test_default_robot_config_values(self):
        """Test default robot configuration values."""
        self.assertEqual(_DEFAULT_ROBOT_CONFIG["leg_length"], 0.58)
        self.assertEqual(_DEFAULT_ROBOT_CONFIG["wheel_radius"], 0.06)
        self.assertEqual(_DEFAULT_ROBOT_CONFIG["mass"], 5.34)
        np.testing.assert_array_equal(
            _DEFAULT_ROBOT_CONFIG["rotation_base_to_imu"],
            np.diag([-1.0, 1.0, -1.0]),
        )

    def test_robot_config_has_expected_structure(self):
        """Test that ROBOT_CONFIG incorporates user configuration properly."""
        # Verify it has the expected keys from default config
        self.assertIn("leg_length", ROBOT_CONFIG)
        self.assertIn("wheel_radius", ROBOT_CONFIG)
        self.assertIn("mass", ROBOT_CONFIG)
        self.assertIn("rotation_base_to_imu", ROBOT_CONFIG)

        # Verify the config values are present (defaults or user overrides)
        self.assertIsInstance(ROBOT_CONFIG["leg_length"], (int, float))
        self.assertIsInstance(ROBOT_CONFIG["wheel_radius"], (int, float))
        self.assertIsInstance(ROBOT_CONFIG["mass"], (int, float))
        self.assertIsInstance(ROBOT_CONFIG["rotation_base_to_imu"], np.ndarray)

        # Test that the structure is preserved from defaults
        self.assertEqual(
            set(ROBOT_CONFIG.keys()), set(_DEFAULT_ROBOT_CONFIG.keys()),
        )

    def test_merge_user_robot_config_no_user_config(self):
        """Test merging when no user config is available."""
        default_config = {"leg_length": 0.3, "mass": 2.5}
        user_config = {}
        result = _merge_user_robot_config(default_config, user_config)
        self.assertEqual(result, default_config)

    def test_merge_user_robot_config_with_overrides(self):
        """Test merging with user config overrides."""
        default_config = {
            "foo": {"bar": 0.1},
            "leg_length": 0.3,
            "mass": 2.5,
        }
        user_config = {
            "robot": {
                "leg_length": 0.35,
                "new_param": 42,
            }
        }
        result = _merge_user_robot_config(default_config, user_config)
        expected = {
            "foo": {"bar": 0.1},  # unchanged
            "leg_length": 0.35,  # overridden
            "mass": 2.5,  # unchanged
            "new_param": 42,  # added
        }
        self.assertEqual(result, expected)

    def test_merge_user_robot_config_nested_override(self):
        """Test merging with nested config overrides."""
        default_config = {
            "foo": {
                "bar": 0.1,
                "testing": 0.1,
                "sailing": 0.05,
            }
        }
        user_config = {"robot": {"foo": {"bar": 0.15}}}  # override only one
        result = _merge_user_robot_config(default_config, user_config)
        expected = {
            "foo": {
                "bar": 0.15,  # overridden
                "testing": 0.1,  # preserved
                "sailing": 0.05,  # preserved
            }
        }
        self.assertEqual(result, expected)

    def test_merge_user_robot_config_no_robot_section(self):
        """Test merging when user config has no robot section."""
        default_config = {"leg_length": 0.3}
        user_config = {"other_section": {"value": 123}}
        result = _merge_user_robot_config(default_config, user_config)
        self.assertEqual(result, default_config)


if __name__ == "__main__":
    unittest.main()
