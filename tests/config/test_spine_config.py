#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from upkie.config.spine_config import (
    _DEFAULT_SPINE_CONFIG,
    SPINE_CONFIG,
    _get_user_config_path,
    _load_yaml_config,
    merge_user_spine_config,
)


class SpineConfigTestCase(unittest.TestCase):
    """!
    Test spine configuration functionality.
    """

    def test_spine_config_root_keys(self):
        """Test that SPINE_CONFIG has expected root keys."""
        self.assertIn("bullet", SPINE_CONFIG)
        self.assertIn("floor_contact", SPINE_CONFIG)
        self.assertIn("wheel_contact", SPINE_CONFIG)
        self.assertIn("wheel_odometry", SPINE_CONFIG)
        self.assertIsInstance(
            SPINE_CONFIG["floor_contact"]["upper_leg_torque_threshold"],
            (int, float),
        )
        self.assertIsInstance(
            SPINE_CONFIG["wheel_contact"]["cutoff_period"], (int, float)
        )

    def test_default_spine_config_structure(self):
        """Test that default spine config has expected structure."""
        self.assertIn("bullet", _DEFAULT_SPINE_CONFIG)
        self.assertIn("floor_contact", _DEFAULT_SPINE_CONFIG)
        self.assertIn("wheel_contact", _DEFAULT_SPINE_CONFIG)
        self.assertIn("wheel_odometry", _DEFAULT_SPINE_CONFIG)

    def test_default_floor_contact_config(self):
        """Test default floor contact configuration."""
        floor_config = _DEFAULT_SPINE_CONFIG["floor_contact"]
        self.assertIn("upper_leg_torque_threshold", floor_config)
        self.assertEqual(floor_config["upper_leg_torque_threshold"], 10.0)

    def test_default_wheel_contact_config(self):
        """Test default wheel contact configuration."""
        wheel_config = _DEFAULT_SPINE_CONFIG["wheel_contact"]
        expected_keys = [
            "cutoff_period",
            "liftoff_inertia",
            "min_touchdown_acceleration",
            "min_touchdown_torque",
            "touchdown_inertia",
        ]
        for key in expected_keys:
            self.assertIn(key, wheel_config)

    def test_default_wheel_odometry_config(self):
        """Test default wheel odometry configuration."""
        odometry_config = _DEFAULT_SPINE_CONFIG["wheel_odometry"]
        self.assertIn("signed_radius", odometry_config)

        radius_config = odometry_config["signed_radius"]
        self.assertIn("left_wheel", radius_config)
        self.assertIn("right_wheel", radius_config)
        self.assertEqual(radius_config["left_wheel"], 0.05)
        self.assertEqual(radius_config["right_wheel"], -0.05)

    def test_merge_user_spine_config_no_user_config(self):
        """Test merging when no user config is available."""
        default_config = {
            "floor_contact": {"upper_leg_torque_threshold": 10.0},
            "wheel_contact": {"cutoff_period": 0.2},
        }
        user_config = {}
        result = merge_user_spine_config(default_config, user_config)
        self.assertEqual(result, default_config)

    def test_merge_user_spine_config_with_overrides(self):
        """Test merging with user config overrides."""
        default_config = {
            "floor_contact": {"upper_leg_torque_threshold": 10.0},
            "wheel_contact": {"cutoff_period": 0.2},
        }
        user_config = {
            "spine": {
                "floor_contact": {"upper_leg_torque_threshold": 25.0},
                "new_section": {"new_value": 42},
            }
        }
        result = merge_user_spine_config(default_config, user_config)
        expected = {
            "floor_contact": {
                "upper_leg_torque_threshold": 25.0
            },  # overridden
            "wheel_contact": {"cutoff_period": 0.2},  # unchanged
            "new_section": {"new_value": 42},  # added
        }
        self.assertEqual(result, expected)

    def test_merge_user_spine_config_nested_override(self):
        """Test merging with nested config overrides."""
        default_config = {
            "wheel_odometry": {
                "signed_radius": {"left_wheel": 0.05, "right_wheel": -0.05}
            }
        }
        user_config = {
            "spine": {
                "wheel_odometry": {
                    "signed_radius": {
                        "left_wheel": 0.06  # only override left wheel
                    }
                }
            }
        }
        result = merge_user_spine_config(default_config, user_config)
        expected = {
            "wheel_odometry": {
                "signed_radius": {
                    "left_wheel": 0.06,  # overridden
                    "right_wheel": -0.05,  # preserved
                }
            }
        }
        self.assertEqual(result, expected)

    def test_merge_user_spine_config_no_spine_section(self):
        """Test merging when user config has no spine section."""
        default_config = {
            "floor_contact": {"upper_leg_torque_threshold": 10.0},
        }
        user_config = {"other_section": {"value": 123}}
        result = merge_user_spine_config(default_config, user_config)
        self.assertEqual(result, default_config)

    def test_get_user_config_path(self):
        """Test that user config path is constructed correctly."""
        path = _get_user_config_path()
        self.assertTrue(path.as_posix().endswith("/.config/upkie/config.yml"))
        self.assertEqual(path.name, "config.yml")

    def test_load_yaml_config_file_not_exists(self):
        """Test loading a configuration file that doesn't exist."""
        with tempfile.TemporaryDirectory() as temp_dir:
            config_path = Path(temp_dir) / "nonexistent.yml"
            result = _load_yaml_config(config_path)
            self.assertDictEqual(result, {})

    @patch("upkie.logging.logger.warning")
    def test_load_yaml_config_invalid_yaml(self, mock_warning):
        """Test loading a configuration file with invalid YAML content."""
        with tempfile.NamedTemporaryFile(
            mode="w", suffix=".yml", delete=False
        ) as f:
            f.write("invalid: yaml: content: [unclosed\n")
            f.flush()
            config_path = Path(f.name)

            result = _load_yaml_config(config_path)
            self.assertDictEqual(result, {})

            # Check that a warning was logged
            mock_warning.assert_called_once()
            call_args = mock_warning.call_args[0][0]
            self.assertTrue(call_args.startswith("Failed to load config from"))

    def test_load_yaml_config_success(self):
        """Test successfully loading a configuration file."""
        with tempfile.NamedTemporaryFile(
            mode="w", suffix=".yml", delete=False
        ) as f:
            f.write(
                "spine:\n"
                "  floor_contact:\n"
                "    upper_leg_torque_threshold: 25.0\n"
            )
            f.flush()
            config_path = Path(f.name)

            result = _load_yaml_config(config_path)
            expected = {
                "spine": {
                    "floor_contact": {"upper_leg_torque_threshold": 25.0}
                }
            }
            self.assertEqual(result, expected)


if __name__ == "__main__":
    unittest.main()
