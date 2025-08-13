#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from upkie.config.user_config import (
    USER_CONFIG,
    _get_user_config_path,
    _load_yaml_config,
)


class TestUserConfig(unittest.TestCase):
    """!
    Test user configuration loading functionality.
    """

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
            self.assertIsNone(result)

    @patch("upkie.config.user_config.logger.warning")  # avoid test output
    def test_load_yaml_config_invalid_yaml(self, mock_warning):
        """Test loading a configuration file with invalid YAML content."""
        with tempfile.NamedTemporaryFile(
            mode="w", suffix=".yml", delete=False
        ) as f:
            f.write("invalid: yaml: content: [unclosed\n")
            f.flush()
            config_path = Path(f.name)

            result = _load_yaml_config(config_path)
            self.assertIsNone(result)

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

    def test_user_config_is_dict(self):
        """Test that USER_CONFIG is loaded as a dictionary."""
        self.assertIsInstance(USER_CONFIG, dict)


if __name__ == "__main__":
    unittest.main()
