#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

"""Tests for UpkieServos environment with SpineBackend."""

import tempfile
import unittest
from multiprocessing.shared_memory import SharedMemory
from pathlib import Path
from unittest.mock import patch

import gymnasium as gym
import numpy as np

from upkie.envs.backends import SpineBackend
from upkie.envs.backends.spine_backend import (
    _DEFAULT_SPINE_CONFIG,
    _get_user_config_path,
    _load_yaml_config,
    merge_user_spine_config,
)
from upkie.envs.testing import MockSpine
from upkie.envs.upkie_servos import UpkieServos
from upkie.exceptions import UpkieTimeoutError


class SpineBackendTestCase(unittest.TestCase):
    def setUp(self):
        shared_memory = SharedMemory(name=None, size=42, create=True)
        self.backend = SpineBackend(shm_name=shared_memory._name)
        self.env = UpkieServos(
            backend=self.backend,
            frequency=100.0,
        )
        shared_memory.close()
        self.backend._spine = MockSpine()

    def test_reset(self):
        observation, info = self.env.reset()
        spine_observation = info["spine_observation"]
        self.assertAlmostEqual(
            observation["left_wheel"]["position"],
            spine_observation["servo"]["left_wheel"]["position"],
        )
        self.assertGreaterEqual(spine_observation["number"], 1)

        right_knee = observation["right_knee"]
        self.assertIsInstance(right_knee["position"], np.ndarray)
        self.assertIsInstance(right_knee["velocity"], np.ndarray)
        self.assertIsInstance(right_knee["torque"], np.ndarray)
        self.assertIsInstance(right_knee["temperature"], np.ndarray)
        self.assertIsInstance(right_knee["voltage"], np.ndarray)
        self.assertEqual(right_knee["position"].shape, (1,))
        self.assertEqual(right_knee["velocity"].shape, (1,))
        self.assertEqual(right_knee["torque"].shape, (1,))
        self.assertEqual(right_knee["temperature"].shape, (1,))
        self.assertEqual(right_knee["voltage"].shape, (1,))

    def test_reward(self):
        self.env.reset()
        action = {
            joint.name: {
                "position": np.nan,
                "velocity": 0.0,
            }
            for joint in self.env.model.joints
        }
        _, reward, _, _, _ = self.env.step(action)
        self.assertAlmostEqual(reward, 1.0)  # survival reward

    def test_action_clamping(self):
        action = {
            joint.name: {
                "position": np.nan,
                "velocity": 0.0,
                "feedforward_torque": 0.0,
            }
            for joint in self.env.model.joints
        }
        not_wheel = "left_hip"  # wheels don't have position limits
        self.env.reset()

        action[not_wheel]["position"] = np.nan
        self.env.step(action)
        self.assertTrue(
            np.isnan(
                self.backend._spine.action["servo"][not_wheel]["position"]
            )
        )

        action[not_wheel]["position"] = 0.5
        self.env.step(action)
        self.assertAlmostEqual(
            self.backend._spine.action["servo"][not_wheel]["position"],
            0.5,
            places=5,
        )

        action[not_wheel]["position"] = 5e5
        self.env.step(action)
        self.assertAlmostEqual(
            self.backend._spine.action["servo"][not_wheel]["position"],
            self.env.action_space[not_wheel]["position"].high[0],
            places=5,
        )

        action[not_wheel]["feedforward_torque"] = 1e20
        self.env.step(action)
        self.assertAlmostEqual(
            self.backend._spine.action["servo"][not_wheel][
                "feedforward_torque"
            ],
            self.env.action_space[not_wheel]["feedforward_torque"].high[0],
            places=5,
        )

    def test_registration(self):
        shm = SharedMemory(name=None, size=42, create=True)
        env = gym.make("Upkie-Spine-Servos", shm_name=shm._name)
        self.assertIsNotNone(env)
        try:
            del env  # we delete it explicitly
        except UpkieTimeoutError:  # to catch this exception
            pass  # which is ok: there is no spine, thus no response
        shm.close()

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
