#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2025 Inria

"""Tests for PyBullet backend friction functionality."""

import unittest
from unittest.mock import MagicMock, patch

import numpy as np

from upkie.envs.backends.pybullet_backend import PyBulletBackend
from upkie.exceptions import UpkieRuntimeError
from upkie.utils.external_force import ExternalForce


class PyBulletBackendTestCase(unittest.TestCase):
    def setUp(self):
        # Mock PyBullet to avoid requiring actual PyBullet installation
        self.pybullet_mock = MagicMock()
        self.pybullet_data_mock = MagicMock()
        self.pybullet_data_mock.getDataPath.return_value = "/mock/path"

        # Mock joint info for Upkie joints
        joint_infos = [
            # (joint_idx, joint_name, link_name)
            (0, b"left_hip", b"left_hip_link"),
            (1, b"left_knee", b"left_knee_link"),
            (2, b"left_wheel", b"left_wheel_link"),
            (3, b"right_hip", b"right_hip_link"),
            (4, b"right_knee", b"right_knee_link"),
            (5, b"right_wheel", b"right_wheel_link"),
            (6, b"imu_joint", b"imu"),
        ]

        def mock_getJointInfo(robot_id, joint_idx):
            if joint_idx < len(joint_infos):
                idx, joint_name, link_name = joint_infos[joint_idx]
                return [
                    None,
                    joint_name,
                    None,
                    None,
                    None,
                    None,
                    None,
                    None,
                    None,
                    None,
                    None,
                    None,
                    link_name,
                ]
            raise IndexError()

        self.pybullet_mock.getJointInfo.side_effect = mock_getJointInfo
        self.pybullet_mock.getNumJoints.return_value = len(joint_infos)
        self.pybullet_mock.connect.return_value = 1
        self.pybullet_mock.loadURDF.return_value = 1
        self.pybullet_mock.GUI = 1
        self.pybullet_mock.DIRECT = 2
        self.pybullet_mock.COV_ENABLE_GUI = 0
        self.pybullet_mock.COV_ENABLE_RENDERING = 1
        self.pybullet_mock.COV_ENABLE_SHADOWS = 2
        self.pybullet_mock.VELOCITY_CONTROL = 1
        self.pybullet_mock.TORQUE_CONTROL = 2
        self.pybullet_mock.WORLD_FRAME = 1
        self.pybullet_mock.LINK_FRAME = 2

        # Mock joint state (position, velocity, force, torque)
        self.pybullet_mock.getJointState.return_value = (0.1, 0.5, 0.0, 0.0)

    @patch("upkie.envs.backends.pybullet_backend.pybullet_data")
    @patch("upkie.envs.backends.pybullet_backend.pybullet")
    @patch("upkie.envs.backends.pybullet_backend.upkie_description")
    def test_joint_properties_initialization(
        self, mock_upkie_desc, mock_pybullet, mock_pybullet_data
    ):
        """Test joint properties are initialized from bullet_config."""
        mock_pybullet.configure_mock(**self.pybullet_mock.__dict__)
        mock_pybullet_data.configure_mock(**self.pybullet_data_mock.__dict__)
        mock_upkie_desc.URDF_PATH = "/mock/urdf/path"

        bullet_config = {
            "torque_control": {"kp": 20.0, "kd": 1.0},
            "joint_properties": {
                "left_hip": {
                    "friction": 0.1,
                    "torque_control_noise": 0.05,
                    "torque_measurement_noise": 0.02,
                },
                "right_knee": {
                    "friction": 0.15,
                },
            },
        }

        backend = PyBulletBackend(
            dt=0.01, bullet_config=bullet_config, gui=False
        )

        # Check that joint properties were initialized correctly
        self.assertEqual(
            backend._joint_properties["left_hip"]["friction"], 0.1
        )
        self.assertEqual(
            backend._joint_properties["left_hip"]["torque_control_noise"], 0.05
        )
        self.assertEqual(
            backend._joint_properties["left_hip"]["torque_measurement_noise"],
            0.02,
        )

        self.assertEqual(
            backend._joint_properties["right_knee"]["friction"], 0.15
        )
        self.assertEqual(
            backend._joint_properties["right_knee"]["torque_control_noise"],
            0.0,
        )  # default
        self.assertEqual(
            backend._joint_properties["right_knee"][
                "torque_measurement_noise"
            ],
            0.0,
        )  # default

        # Joint without specific config should have default values
        self.assertEqual(
            backend._joint_properties["left_knee"]["friction"], 0.0
        )
        self.assertEqual(
            backend._joint_properties["left_knee"]["torque_control_noise"], 0.0
        )
        self.assertEqual(
            backend._joint_properties["left_knee"]["torque_measurement_noise"],
            0.0,
        )

        backend.close()

    @patch("upkie.envs.backends.pybullet_backend.pybullet_data")
    @patch("upkie.envs.backends.pybullet_backend.pybullet")
    @patch("upkie.envs.backends.pybullet_backend.upkie_description")
    def test_friction_computation_no_friction(
        self, mock_upkie_desc, mock_pybullet, mock_pybullet_data
    ):
        """Test joint torque computation without friction."""
        mock_pybullet.configure_mock(**self.pybullet_mock.__dict__)
        mock_pybullet_data.configure_mock(**self.pybullet_data_mock.__dict__)
        mock_upkie_desc.URDF_PATH = "/mock/urdf/path"

        bullet_config = {
            "torque_control": {"kp": 20.0, "kd": 1.0},
            "joint_properties": {"left_hip": {"friction": 0.0}},  # No friction
        }

        backend = PyBulletBackend(
            dt=0.01, bullet_config=bullet_config, gui=False
        )

        # Test with measured velocity above stiction threshold
        self.pybullet_mock.getJointState.return_value = (
            0.1,
            0.005,
            0.0,
            0.0,
        )  # velocity = 0.005 > 1e-3

        torque = backend.compute_joint_torque(
            joint_name="left_hip",
            feedforward_torque=1.0,
            target_position=0.0,
            target_velocity=0.0,
            kp_scale=1.0,
            kd_scale=1.0,
            maximum_torque=10.0,
        )

        # Expected: ff +
        #     kd * (target_vel - measured_vel) +
        #     kp * (target_pos - measured_pos)
        # = 1.0 + 1.0 * (0.0 - 0.005) + 20.0 * (0.0 - 0.1)
        # = 1.0 - 0.005 - 2.0 = -1.005
        expected_torque = 1.0 + 1.0 * (0.0 - 0.005) + 20.0 * (0.0 - 0.1)
        self.assertAlmostEqual(torque, expected_torque, places=5)

        backend.close()

    @patch("upkie.envs.backends.pybullet_backend.pybullet_data")
    @patch("upkie.envs.backends.pybullet_backend.pybullet")
    @patch("upkie.envs.backends.pybullet_backend.upkie_description")
    def test_friction_computation_with_friction(
        self, mock_upkie_desc, mock_pybullet, mock_pybullet_data
    ):
        """Test joint torque computation with friction."""
        mock_pybullet.configure_mock(**self.pybullet_mock.__dict__)
        mock_pybullet_data.configure_mock(**self.pybullet_data_mock.__dict__)
        mock_upkie_desc.URDF_PATH = "/mock/urdf/path"

        friction_value = 0.1
        bullet_config = {
            "torque_control": {"kp": 20.0, "kd": 1.0},
            "joint_properties": {"left_hip": {"friction": friction_value}},
        }

        backend = PyBulletBackend(
            dt=0.01, bullet_config=bullet_config, gui=False
        )

        # Test positive velocity (friction opposes motion)
        self.pybullet_mock.getJointState.return_value = (
            0.0,
            0.005,
            0.0,
            0.0,
        )  # velocity = 0.005 > 1e-3

        torque_positive = backend.compute_joint_torque(
            joint_name="left_hip",
            feedforward_torque=0.0,
            target_position=0.0,
            target_velocity=0.0,
            kp_scale=1.0,
            kd_scale=1.0,
            maximum_torque=10.0,
        )

        # Expected: 0.0 + 1.0 * (0.0 - 0.005) + 20.0 * (0.0 - 0.0) +
        #     friction_opposing_motion
        # = -0.005 + (-0.1) = -0.105 (friction opposes positive velocity)
        expected_positive = (
            0.0 + 1.0 * (0.0 - 0.005) + 20.0 * (0.0 - 0.0) - friction_value
        )
        self.assertAlmostEqual(torque_positive, expected_positive, places=5)

        # Test negative velocity (friction still opposes motion)
        self.pybullet_mock.getJointState.return_value = (
            0.0,
            -0.005,
            0.0,
            0.0,
        )  # velocity = -0.005 < -1e-3

        torque_negative = backend.compute_joint_torque(
            joint_name="left_hip",
            feedforward_torque=0.0,
            target_position=0.0,
            target_velocity=0.0,
            kp_scale=1.0,
            kd_scale=1.0,
            maximum_torque=10.0,
        )

        # Expected: 0.0 + 1.0 * (0.0 - (-0.005)) + 20.0 * (0.0 - 0.0) +
        #     friction_opposing_motion
        # = 0.005 + 0.1 = 0.105 (friction opposes negative velocity)
        expected_negative = (
            0.0 + 1.0 * (0.0 - (-0.005)) + 20.0 * (0.0 - 0.0) + friction_value
        )
        self.assertAlmostEqual(torque_negative, expected_negative, places=5)

        backend.close()

    @patch("upkie.envs.backends.pybullet_backend.pybullet_data")
    @patch("upkie.envs.backends.pybullet_backend.pybullet")
    @patch("upkie.envs.backends.pybullet_backend.upkie_description")
    def test_friction_stiction_threshold(
        self, mock_upkie_desc, mock_pybullet, mock_pybullet_data
    ):
        """No friction below stiction velocity threshold."""
        mock_pybullet.configure_mock(**self.pybullet_mock.__dict__)
        mock_pybullet_data.configure_mock(**self.pybullet_data_mock.__dict__)
        mock_upkie_desc.URDF_PATH = "/mock/urdf/path"

        friction_value = 0.1
        bullet_config = {
            "torque_control": {"kp": 20.0, "kd": 1.0},
            "joint_properties": {"left_hip": {"friction": friction_value}},
        }

        backend = PyBulletBackend(
            dt=0.01, bullet_config=bullet_config, gui=False
        )

        # Test velocity below stiction threshold
        self.pybullet_mock.getJointState.return_value = (
            0.0,
            0.0005,
            0.0,
            0.0,
        )  # velocity = 0.0005 < 1e-3

        torque_below_threshold = backend.compute_joint_torque(
            joint_name="left_hip",
            feedforward_torque=0.0,
            target_position=0.0,
            target_velocity=0.0,
            kp_scale=1.0,
            kd_scale=1.0,
            maximum_torque=10.0,
        )

        # Expected: no friction should be applied
        expected = 0.0 + 1.0 * (0.0 - 0.0005) + 20.0 * (0.0 - 0.0)
        self.assertAlmostEqual(torque_below_threshold, expected, places=5)

        # Test velocity exactly at stiction threshold
        self.pybullet_mock.getJointState.return_value = (
            0.0,
            1e-3,
            0.0,
            0.0,
        )  # velocity = 1e-3

        torque_at_threshold = backend.compute_joint_torque(
            joint_name="left_hip",
            feedforward_torque=0.0,
            target_position=0.0,
            target_velocity=0.0,
            kp_scale=1.0,
            kd_scale=1.0,
            maximum_torque=10.0,
        )

        # Expected: no friction should be applied (velocity not > 1e-3)
        expected = 0.0 + 1.0 * (0.0 - 1e-3) + 20.0 * (0.0 - 0.0)
        self.assertAlmostEqual(torque_at_threshold, expected, places=5)

        backend.close()

    @patch("upkie.envs.backends.pybullet_backend.pybullet_data")
    @patch("upkie.envs.backends.pybullet_backend.pybullet")
    @patch("upkie.envs.backends.pybullet_backend.upkie_description")
    def test_torque_measurement_noise(
        self, mock_upkie_desc, mock_pybullet, mock_pybullet_data
    ):
        """Test torque measurement noise is applied correctly."""
        mock_pybullet.configure_mock(**self.pybullet_mock.__dict__)
        mock_pybullet_data.configure_mock(**self.pybullet_data_mock.__dict__)
        mock_upkie_desc.URDF_PATH = "/mock/urdf/path"

        # Mock base pose and velocity for get_spine_observation
        mock_pybullet.getBasePositionAndOrientation.return_value = (
            [0.0, 0.0, 0.6],
            [0.0, 0.0, 0.0, 1.0],
        )
        mock_pybullet.getBaseVelocity.return_value = (
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
        )
        mock_pybullet.getLinkState.return_value = [
            None,
            None,
            None,
            None,
            None,
            [0.0, 0.0, 0.0, 1.0],  # link orientation
            [0.0, 0.0, 0.0],  # linear velocity
            [0.0, 0.0, 0.0],  # angular velocity
        ]

        bullet_config = {
            "torque_control": {"kp": 20.0, "kd": 1.0},
            "joint_properties": {
                "left_hip": {
                    "torque_measurement_noise": 0.1,  # Significant noise
                },
                "right_hip": {
                    "torque_measurement_noise": 0.0,  # No noise
                },
            },
        }

        backend = PyBulletBackend(
            dt=0.01, bullet_config=bullet_config, gui=False
        )

        # Simulate a servo action to generate torques
        servo_action = {
            "position": 0.1,
            "velocity": 0.0,
            "kp_scale": 1.0,
            "kd_scale": 1.0,
            "maximum_torque": 10.0,
        }
        action = {
            "servo": {"left_hip": servo_action, "right_hip": servo_action}
        }

        # Set up joint state for torque computation
        self.pybullet_mock.getJointState.return_value = (0.0, 0.0, 0.0, 0.0)

        # Get multiple observations and check noise characteristics
        observations = []
        for _ in range(100):
            # Step the simulation to compute and store torques
            backend.step(action)
            obs = backend.get_spine_observation()
            observations.append(obs)

        # Extract torque values for joints with and without noise
        left_hip_torques = [
            obs["servo"]["left_hip"]["torque"] for obs in observations
        ]
        right_hip_torques = [
            obs["servo"]["right_hip"]["torque"] for obs in observations
        ]

        # The expected computed torque is kp * (target_position -
        # measured_position), no kd term since both velocities are zero
        expected_torque = 20.0 * (0.1 - 0.0)

        # Joint without noise should have constant torque
        self.assertTrue(
            all(
                abs(torque - expected_torque) < 1e-10
                for torque in right_hip_torques
            ),
            "Right hip torque should be constant (no noise configured)",
        )

        # Joint with noise should have varying torque values
        left_hip_std = np.std(left_hip_torques)
        left_hip_mean = np.mean(left_hip_torques)

        # Check that noise has reasonable statistics
        # Mean should be close to expected torque (unbiased noise)
        self.assertAlmostEqual(left_hip_mean, expected_torque, delta=0.05)

        # Standard deviation should be close to configured noise level
        self.assertGreater(
            left_hip_std, 0.05, "Left hip torque should vary due to noise"
        )
        self.assertLess(
            left_hip_std, 0.15, "Noise magnitude should be reasonable"
        )

        # Check that different samples have different values
        unique_torques = set(round(t, 6) for t in left_hip_torques[:10])
        self.assertGreater(
            len(unique_torques),
            5,
            "Should get different torque values due to noise",
        )

        backend.close()

    @patch("upkie.envs.backends.pybullet_backend.pybullet_data")
    @patch("upkie.envs.backends.pybullet_backend.pybullet")
    @patch("upkie.envs.backends.pybullet_backend.upkie_description")
    def test_torque_measurement_noise_threshold(
        self, mock_upkie_desc, mock_pybullet, mock_pybullet_data
    ):
        """Test that small noise values are ignored (threshold behavior)."""
        mock_pybullet.configure_mock(**self.pybullet_mock.__dict__)
        mock_pybullet_data.configure_mock(**self.pybullet_data_mock.__dict__)
        mock_upkie_desc.URDF_PATH = "/mock/urdf/path"

        # Mock required methods for get_spine_observation
        mock_pybullet.getBasePositionAndOrientation.return_value = (
            [0.0, 0.0, 0.6],
            [0.0, 0.0, 0.0, 1.0],
        )
        mock_pybullet.getBaseVelocity.return_value = (
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
        )
        mock_pybullet.getLinkState.return_value = [
            None,
            None,
            None,
            None,
            None,
            [0.0, 0.0, 0.0, 1.0],  # link orientation
            [0.0, 0.0, 0.0],  # linear velocity
            [0.0, 0.0, 0.0],  # angular velocity
        ]

        bullet_config = {
            "torque_control": {"kp": 20.0, "kd": 1.0},
            "joint_properties": {
                "left_hip": {
                    "torque_measurement_noise": 1e-11,  # Below threshold
                },
            },
        }

        backend = PyBulletBackend(
            dt=0.01, bullet_config=bullet_config, gui=False
        )

        # Simulate a servo action to generate torques
        servo_action = {
            "position": 0.05,
            "velocity": 0.0,
            "kp_scale": 1.0,
            "kd_scale": 1.0,
            "maximum_torque": 10.0,
        }
        action = {"servo": {"left_hip": servo_action}}

        self.pybullet_mock.getJointState.return_value = (0.0, 0.0, 0.0, 0.0)

        # Get multiple observations
        observations = []
        for _ in range(50):
            backend.step(action)
            obs = backend.get_spine_observation()
            observations.append(obs)

        # All torque values should be exactly the same (no noise applied)
        left_hip_torques = [
            obs["servo"]["left_hip"]["torque"] for obs in observations
        ]

        # Expected torque: kp * (target_position - measured_position)
        expected_torque = 20.0 * (0.05 - 0.0)

        self.assertTrue(
            all(
                abs(torque - expected_torque) < 1e-10
                for torque in left_hip_torques
            ),
            "Torque should be constant when noise is below threshold",
        )

        backend.close()

    @patch("upkie.envs.backends.pybullet_backend.pybullet_data")
    @patch("upkie.envs.backends.pybullet_backend.pybullet")
    @patch("upkie.envs.backends.pybullet_backend.upkie_description")
    def test_torques_are_nonzero_with_actions(
        self, mock_upkie_desc, mock_pybullet, mock_pybullet_data
    ):
        """Test that torques are non-zero when actions are applied."""
        mock_pybullet.configure_mock(**self.pybullet_mock.__dict__)
        mock_pybullet_data.configure_mock(**self.pybullet_data_mock.__dict__)
        mock_upkie_desc.URDF_PATH = "/mock/urdf/path"

        # Mock required methods for get_spine_observation
        mock_pybullet.getBasePositionAndOrientation.return_value = (
            [0.0, 0.0, 0.6],
            [0.0, 0.0, 0.0, 1.0],
        )
        mock_pybullet.getBaseVelocity.return_value = (
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
        )
        mock_pybullet.getLinkState.return_value = [
            None,
            None,
            None,
            None,
            None,
            [0.0, 0.0, 0.0, 1.0],  # link orientation
            [0.0, 0.0, 0.0],  # linear velocity
            [0.0, 0.0, 0.0],  # angular velocity
        ]

        bullet_config = {
            "torque_control": {"kp": 20.0, "kd": 1.0},
            "joint_properties": {
                "left_hip": {"torque_measurement_noise": 0.0},
                "left_knee": {"torque_measurement_noise": 0.0},
            },
        }

        backend = PyBulletBackend(
            dt=0.01, bullet_config=bullet_config, gui=False
        )

        # Initially, torques should be zero (no actions applied yet)
        initial_obs = backend.get_spine_observation()
        self.assertEqual(
            initial_obs["servo"]["left_hip"]["torque"],
            0.0,
            "Initial torque should be zero before any actions",
        )
        self.assertEqual(
            initial_obs["servo"]["left_knee"]["torque"],
            0.0,
            "Initial torque should be zero before any actions",
        )

        # Apply a servo action that should generate non-zero torques
        servo_action = {
            "position": 0.2,  # Non-zero target position
            "velocity": 0.1,  # Non-zero target velocity
            "kp_scale": 1.0,
            "kd_scale": 1.0,
            "maximum_torque": 10.0,
        }
        action = {
            "servo": {"left_hip": servo_action, "left_knee": servo_action}
        }

        # Set joint state with position and velocity different from targets
        self.pybullet_mock.getJointState.return_value = (0.1, 0.05, 0.0, 0.0)

        # Step the simulation with the action
        backend.step(action)
        obs_after_action = backend.get_spine_observation()

        # Torques should now be non-zero
        left_hip_torque = obs_after_action["servo"]["left_hip"]["torque"]
        left_knee_torque = obs_after_action["servo"]["left_knee"]["torque"]

        self.assertNotEqual(
            left_hip_torque,
            0.0,
            "Left hip torque should be non-zero after applying action",
        )
        self.assertNotEqual(
            left_knee_torque,
            0.0,
            "Left knee torque should be non-zero after applying action",
        )

        # Verify torques are reasonable (not NaN or infinite)
        self.assertTrue(
            abs(left_hip_torque) < 100.0,
            f"Left hip torque should be reasonable, got {left_hip_torque}",
        )
        self.assertTrue(
            abs(left_knee_torque) < 100.0,
            f"Left knee torque should be reasonable, got {left_knee_torque}",
        )

        # Expected torque calculation:
        # kp * (target_pos - measured_pos) + kd * (target_vel - measured_vel)
        # = 20.0 * (0.2 - 0.1) + 1.0 * (0.1 - 0.05) = 2.0 + 0.05 = 2.05
        expected_torque = 20.0 * (0.2 - 0.1) + 1.0 * (0.1 - 0.05)
        self.assertAlmostEqual(
            left_hip_torque,
            expected_torque,
            places=5,
            msg="Torque should match expected calculation",
        )
        self.assertAlmostEqual(
            left_knee_torque,
            expected_torque,
            places=5,
            msg="Torque should match expected calculation",
        )

        backend.close()

    @patch("upkie.envs.backends.pybullet_backend.pybullet_data")
    @patch("upkie.envs.backends.pybullet_backend.pybullet")
    @patch("upkie.envs.backends.pybullet_backend.upkie_description")
    def test_torque_control_noise(
        self, mock_upkie_desc, mock_pybullet, mock_pybullet_data
    ):
        """Test torque control noise is applied during torque computation."""
        mock_pybullet.configure_mock(**self.pybullet_mock.__dict__)
        mock_pybullet_data.configure_mock(**self.pybullet_data_mock.__dict__)
        mock_upkie_desc.URDF_PATH = "/mock/urdf/path"

        bullet_config = {
            "torque_control": {"kp": 20.0, "kd": 1.0},
            "joint_properties": {
                "left_hip": {
                    "torque_control_noise": 0.1,  # Significant noise
                },
                "right_hip": {
                    "torque_control_noise": 0.0,  # No noise
                },
            },
        }

        backend = PyBulletBackend(
            dt=0.01, bullet_config=bullet_config, gui=False
        )

        # Set up joint state for torque computation
        self.pybullet_mock.getJointState.return_value = (0.1, 0.05, 0.0, 0.0)

        # Test parameters for torque computation
        target_position = 0.2
        target_velocity = 0.1
        feedforward_torque = 0.5
        maximum_torque = 10.0

        # Compute multiple torques and check noise characteristics
        left_hip_torques = []
        right_hip_torques = []
        for _ in range(100):
            left_hip_torque = backend.compute_joint_torque(
                "left_hip",
                feedforward_torque,
                target_position,
                target_velocity,
                1.0,  # kp_scale
                1.0,  # kd_scale
                maximum_torque,
            )
            right_hip_torque = backend.compute_joint_torque(
                "right_hip",
                feedforward_torque,
                target_position,
                target_velocity,
                1.0,  # kp_scale
                1.0,  # kd_scale
                maximum_torque,
            )
            left_hip_torques.append(left_hip_torque)
            right_hip_torques.append(right_hip_torque)

        # Expected base torque: feedforward + kd*(target_vel - measured_vel) +
        # kp*(target_pos - measured_pos)
        expected_base_torque = (
            feedforward_torque
            + 1.0 * (target_velocity - 0.05)
            + 20.0 * (target_position - 0.1)
        )

        # Joint without noise should have constant torque
        self.assertTrue(
            all(
                abs(torque - expected_base_torque) < 1e-10
                for torque in right_hip_torques
            ),
            "Right hip torque should be constant (no control noise)",
        )

        # Joint with control noise should have varying torque values
        left_hip_std = np.std(left_hip_torques)
        left_hip_mean = np.mean(left_hip_torques)

        # Check that noise has reasonable statistics
        # Mean should be close to expected torque (unbiased noise)
        self.assertAlmostEqual(left_hip_mean, expected_base_torque, delta=0.05)

        # Standard deviation should be close to configured noise level
        self.assertGreater(
            left_hip_std,
            0.05,
            "Left hip torque should vary due to control noise",
        )
        self.assertLess(
            left_hip_std, 0.15, "Control noise magnitude should be reasonable"
        )

        # Check that different samples have different values
        unique_torques = set(round(t, 6) for t in left_hip_torques[:10])
        self.assertGreater(
            len(unique_torques),
            5,
            "Should get different torque values due to control noise",
        )

        backend.close()

    @patch("upkie.envs.backends.pybullet_backend.pybullet_data")
    @patch("upkie.envs.backends.pybullet_backend.pybullet")
    @patch("upkie.envs.backends.pybullet_backend.upkie_description")
    def test_torque_control_noise_threshold(
        self, mock_upkie_desc, mock_pybullet, mock_pybullet_data
    ):
        """Test that small torque control noise values are ignored."""
        mock_pybullet.configure_mock(**self.pybullet_mock.__dict__)
        mock_pybullet_data.configure_mock(**self.pybullet_data_mock.__dict__)
        mock_upkie_desc.URDF_PATH = "/mock/urdf/path"

        bullet_config = {
            "torque_control": {"kp": 20.0, "kd": 1.0},
            "joint_properties": {
                "left_hip": {
                    "torque_control_noise": 1e-11,  # Below threshold
                },
            },
        }

        backend = PyBulletBackend(
            dt=0.01, bullet_config=bullet_config, gui=False
        )

        # Set up joint state for torque computation
        self.pybullet_mock.getJointState.return_value = (0.0, 0.0, 0.0, 0.0)

        # Test parameters
        target_position = 0.05
        feedforward_torque = 1.0
        maximum_torque = 10.0

        # Compute multiple torques
        torques = []
        for _ in range(50):
            torque = backend.compute_joint_torque(
                "left_hip",
                feedforward_torque,
                target_position,
                0.0,  # target_velocity
                1.0,  # kp_scale
                1.0,  # kd_scale
                maximum_torque,
            )
            torques.append(torque)

        # Expected torque: feedforward + kp * (target_pos - measured_pos)
        # = 1.0 + 20.0 * (0.05 - 0.0) = 1.0 + 1.0 = 2.0
        expected_torque = feedforward_torque + 20.0 * (target_position - 0.0)

        # All torque values should be exactly the same (no noise applied)
        self.assertTrue(
            all(abs(torque - expected_torque) < 1e-10 for torque in torques),
            "Torque should be constant when control noise is below threshold",
        )

        backend.close()

    @patch("upkie.envs.backends.pybullet_backend.pybullet_data")
    @patch("upkie.envs.backends.pybullet_backend.pybullet")
    @patch("upkie.envs.backends.pybullet_backend.upkie_description")
    def test_control_vs_measurement_noise_independence(
        self, mock_upkie_desc, mock_pybullet, mock_pybullet_data
    ):
        """Test that control noise and measurement noise are independent."""
        mock_pybullet.configure_mock(**self.pybullet_mock.__dict__)
        mock_pybullet_data.configure_mock(**self.pybullet_data_mock.__dict__)
        mock_upkie_desc.URDF_PATH = "/mock/urdf/path"

        # Mock required methods for get_spine_observation
        mock_pybullet.getBasePositionAndOrientation.return_value = (
            [0.0, 0.0, 0.6],
            [0.0, 0.0, 0.0, 1.0],
        )
        mock_pybullet.getBaseVelocity.return_value = (
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
        )
        mock_pybullet.getLinkState.return_value = [
            None,
            None,
            None,
            None,
            None,
            [0.0, 0.0, 0.0, 1.0],  # link orientation
            [0.0, 0.0, 0.0],  # linear velocity
            [0.0, 0.0, 0.0],  # angular velocity
        ]

        bullet_config = {
            "torque_control": {"kp": 20.0, "kd": 1.0},
            "joint_properties": {
                "left_hip": {
                    "torque_control_noise": 0.1,  # Control noise
                    "torque_measurement_noise": 0.05,  # Measurement noise
                },
            },
        }

        backend = PyBulletBackend(
            dt=0.01, bullet_config=bullet_config, gui=False
        )

        # Set up joint state
        self.pybullet_mock.getJointState.return_value = (0.0, 0.0, 0.0, 0.0)

        # Apply action and get observations multiple times
        servo_action = {
            "position": 0.1,
            "velocity": 0.0,
            "kp_scale": 1.0,
            "kd_scale": 1.0,
            "maximum_torque": 10.0,
        }
        action = {"servo": {"left_hip": servo_action}}

        stored_torques = []  # These should have control noise
        observed_torques = []  # These should have control + measurement noise

        for _ in range(100):
            # Step to compute and store torque (with control noise)
            backend.step(action)
            stored_torque = backend._PyBulletBackend__joint_torques["left_hip"]

            # Get observation (adds measurement noise on top)
            obs = backend.get_spine_observation()
            observed_torque = obs["servo"]["left_hip"]["torque"]

            stored_torques.append(stored_torque)
            observed_torques.append(observed_torque)

        # Both should vary, but observed torques should have more variance
        stored_std = np.std(stored_torques)
        observed_std = np.std(observed_torques)

        # Control noise should cause variation in stored torques
        self.assertGreater(
            stored_std, 0.05, "Stored torques should vary due to control noise"
        )

        # Observed torques should differ from measurement noise
        self.assertGreater(
            observed_std,
            stored_std,
            "Observed torques should have more variance "
            "due to measurement noise",
        )

        backend.close()

    @patch("upkie.envs.backends.pybullet_backend.pybullet_data")
    @patch("upkie.envs.backends.pybullet_backend.pybullet")
    @patch("upkie.envs.backends.pybullet_backend.upkie_description")
    def test_inertia_randomization_initialization(
        self, mock_upkie_desc, mock_pybullet, mock_pybullet_data
    ):
        """Test that inertia randomization is applied during initialization."""
        mock_pybullet.configure_mock(**self.pybullet_mock.__dict__)
        mock_pybullet_data.configure_mock(**self.pybullet_data_mock.__dict__)
        mock_upkie_desc.URDF_PATH = "/mock/urdf/path"

        # Mock getDynamicsInfo to return consistent test data for all joints
        # Use the same joint count as the existing mock setup (7 joints)
        expected_joint_count = 7  # From the base setUp method
        test_dynamics = []
        for i in range(expected_joint_count):
            mass = 1.0 + i * 0.5  # Different masses for each link
            inertia_val = 0.1 + i * 0.05  # Different inertias for each link
            test_dynamics.append(
                (mass, [0, 0, 0], [inertia_val, inertia_val, inertia_val])
            )

        def mock_getDynamicsInfo(robot_id, link_id):
            if link_id < len(test_dynamics):
                return test_dynamics[link_id]
            return (0.0, [0, 0, 0], [0.0, 0.0, 0.0])

        mock_pybullet.getDynamicsInfo.side_effect = mock_getDynamicsInfo
        # Don't override getNumJoints - use the existing mock setup

        bullet_config = {
            "torque_control": {"kp": 20.0, "kd": 1.0},
            "inertia_variation": 0.2,  # 20% variation
        }

        # Create backend (should trigger inertia randomization)
        backend = PyBulletBackend(
            dt=0.01, bullet_config=bullet_config, gui=False
        )

        # Check that changeDynamics was called for each link
        self.assertEqual(
            mock_pybullet.changeDynamics.call_count,
            expected_joint_count,
            "changeDynamics should be called once per link",
        )

        # Verify that changes were applied with reasonable variations
        for i, call_args in enumerate(
            mock_pybullet.changeDynamics.call_args_list
        ):
            args, kwargs = call_args
            robot_id, link_id = args[:2]

            self.assertEqual(robot_id, backend._robot_id)
            self.assertEqual(link_id, i)
            self.assertIn("mass", kwargs)
            self.assertIn("localInertiaDiagonal", kwargs)

            # Check that mass and inertia have been modified
            original_mass = test_dynamics[i][0]
            original_inertia = test_dynamics[i][2]
            new_mass = kwargs["mass"]
            new_inertia = kwargs["localInertiaDiagonal"]

            # Mass should be within reasonable bounds: original * (1 ± 0.2)
            self.assertGreater(new_mass, original_mass * 0.8)
            self.assertLess(new_mass, original_mass * 1.2)

            # Inertia components should also be modified proportionally
            for j in range(3):
                self.assertGreater(new_inertia[j], original_inertia[j] * 0.8)
                self.assertLess(new_inertia[j], original_inertia[j] * 1.2)

        backend.close()

    @patch("upkie.envs.backends.pybullet_backend.pybullet_data")
    @patch("upkie.envs.backends.pybullet_backend.pybullet")
    @patch("upkie.envs.backends.pybullet_backend.upkie_description")
    def test_inertia_randomization_no_variation(
        self, mock_upkie_desc, mock_pybullet, mock_pybullet_data
    ):
        """Test that no randomization happens when inertia_variation is 0."""
        mock_pybullet.configure_mock(**self.pybullet_mock.__dict__)
        mock_pybullet_data.configure_mock(**self.pybullet_data_mock.__dict__)
        mock_upkie_desc.URDF_PATH = "/mock/urdf/path"

        # Mock getDynamicsInfo
        mock_pybullet.getDynamicsInfo.return_value = (
            1.0,
            [0, 0, 0],
            [0.1, 0.1, 0.1],
        )
        # Use existing mock joint count

        bullet_config = {
            "torque_control": {"kp": 20.0, "kd": 1.0},
            "inertia_variation": 0.0,  # No variation
        }

        # Create backend
        backend = PyBulletBackend(
            dt=0.01, bullet_config=bullet_config, gui=False
        )

        # changeDynamics should not have been called
        self.assertEqual(
            mock_pybullet.changeDynamics.call_count,
            0,
            "changeDynamics should not be called when inertia_variation is 0",
        )

        backend.close()

    @patch("upkie.envs.backends.pybullet_backend.pybullet_data")
    @patch("upkie.envs.backends.pybullet_backend.pybullet")
    @patch("upkie.envs.backends.pybullet_backend.upkie_description")
    def test_inertia_randomization_small_threshold(
        self, mock_upkie_desc, mock_pybullet, mock_pybullet_data
    ):
        """Test that tiny inertia_variation values are ignored."""
        mock_pybullet.configure_mock(**self.pybullet_mock.__dict__)
        mock_pybullet_data.configure_mock(**self.pybullet_data_mock.__dict__)
        mock_upkie_desc.URDF_PATH = "/mock/urdf/path"

        mock_pybullet.getDynamicsInfo.return_value = (
            1.0,
            [0, 0, 0],
            [0.1, 0.1, 0.1],
        )
        # Use existing mock joint count

        bullet_config = {
            "torque_control": {"kp": 20.0, "kd": 1.0},
            "inertia_variation": 1e-11,  # Below threshold (1e-10)
        }

        backend = PyBulletBackend(
            dt=0.01, bullet_config=bullet_config, gui=False
        )

        # changeDynamics should not have been called
        self.assertEqual(
            mock_pybullet.changeDynamics.call_count,
            0,
            "changeDynamics shouldn't be called when inertia_variation is low",
        )

        backend.close()

    @patch("upkie.envs.backends.pybullet_backend.pybullet_data")
    @patch("upkie.envs.backends.pybullet_backend.pybullet")
    @patch("upkie.envs.backends.pybullet_backend.upkie_description")
    def test_randomize_inertias_method(
        self, mock_upkie_desc, mock_pybullet, mock_pybullet_data
    ):
        """Test the randomize_inertias method can be called independently."""
        mock_pybullet.configure_mock(**self.pybullet_mock.__dict__)
        mock_pybullet_data.configure_mock(**self.pybullet_data_mock.__dict__)
        mock_upkie_desc.URDF_PATH = "/mock/urdf/path"

        # Mock dynamics info for all 7 links to match the base setUp
        test_dynamics = [
            (3.0, [0, 0, 0], [0.3, 0.25, 0.35]),  # link 0
            (1.2, [0, 0, 0], [0.12, 0.15, 0.18]),  # link 1
            (2.5, [0, 0, 0], [0.25, 0.20, 0.30]),  # link 2
            (1.8, [0, 0, 0], [0.18, 0.22, 0.16]),  # link 3
            (2.1, [0, 0, 0], [0.21, 0.19, 0.23]),  # link 4
            (1.6, [0, 0, 0], [0.16, 0.17, 0.15]),  # link 5
            (1.4, [0, 0, 0], [0.14, 0.13, 0.12]),  # link 6
        ]

        def mock_getDynamicsInfo(robot_id, link_id):
            if link_id < len(test_dynamics):
                return test_dynamics[link_id]
            return (0.0, [0, 0, 0], [0.0, 0.0, 0.0])

        mock_pybullet.getDynamicsInfo.side_effect = mock_getDynamicsInfo
        # Use existing mock joint count

        # Create backend without automatic randomization
        bullet_config = {
            "torque_control": {"kp": 20.0, "kd": 1.0},
            "inertia_variation": 0.0,  # No automatic randomization
        }

        backend = PyBulletBackend(
            dt=0.01, bullet_config=bullet_config, gui=False
        )

        # Clear any calls from initialization
        mock_pybullet.changeDynamics.reset_mock()

        # Call randomize_inertias manually
        variation = 0.15  # 15% variation
        backend.randomize_inertias(variation)

        # Should have called changeDynamics for each link
        expected_calls = 7  # Based on the base setUp mock joint count
        self.assertEqual(
            mock_pybullet.changeDynamics.call_count,
            expected_calls,
            "randomize_inertias should modify dynamics of all links",
        )

        # Verify the parameters passed to changeDynamics
        for i, call_args in enumerate(
            mock_pybullet.changeDynamics.call_args_list
        ):
            args, kwargs = call_args
            robot_id, link_id = args[:2]

            self.assertEqual(robot_id, backend._robot_id)
            self.assertEqual(link_id, i)
            self.assertIn("mass", kwargs)
            self.assertIn("localInertiaDiagonal", kwargs)

            original_mass = test_dynamics[i][0]
            original_inertia = test_dynamics[i][2]

            # Verify variations are within expected bounds
            new_mass = kwargs["mass"]
            new_inertia = kwargs["localInertiaDiagonal"]

            self.assertGreater(new_mass, original_mass * (1 - variation))
            self.assertLess(new_mass, original_mass * (1 + variation))

            for j in range(3):
                self.assertGreater(
                    new_inertia[j], original_inertia[j] * (1 - variation)
                )
                self.assertLess(
                    new_inertia[j], original_inertia[j] * (1 + variation)
                )

        backend.close()

    @patch("upkie.envs.backends.pybullet_backend.pybullet_data")
    @patch("upkie.envs.backends.pybullet_backend.pybullet")
    @patch("upkie.envs.backends.pybullet_backend.upkie_description")
    def test_randomize_inertias_repeatability(
        self, mock_upkie_desc, mock_pybullet, mock_pybullet_data
    ):
        """Test that repeated calls produce different results."""
        mock_pybullet.configure_mock(**self.pybullet_mock.__dict__)
        mock_pybullet_data.configure_mock(**self.pybullet_data_mock.__dict__)
        mock_upkie_desc.URDF_PATH = "/mock/urdf/path"

        mock_pybullet.getDynamicsInfo.return_value = (
            1.0,
            [0, 0, 0],
            [0.1, 0.1, 0.1],
        )
        # Use existing mock joint count

        bullet_config = {
            "torque_control": {"kp": 20.0, "kd": 1.0},
            "inertia_variation": 0.0,  # No automatic randomization
        }

        backend = PyBulletBackend(
            dt=0.01, bullet_config=bullet_config, gui=False
        )

        # Clear initialization calls
        mock_pybullet.changeDynamics.reset_mock()

        # Call randomize_inertias multiple times and collect results
        masses = []
        inertias = []
        variation = 0.2

        for _ in range(10):
            backend.randomize_inertias(variation)
            # Get the most recent call
            call_args = mock_pybullet.changeDynamics.call_args_list[-1]
            _, kwargs = call_args
            masses.append(kwargs["mass"])
            inertias.append(kwargs["localInertiaDiagonal"])

        # Should have different values (very unlikely to be all the same)
        unique_masses = set(round(m, 8) for m in masses)
        self.assertGreater(
            len(unique_masses),
            5,
            "Should get different mass values from repeated randomization",
        )

        # Check that all values are within reasonable bounds
        for mass in masses:
            self.assertGreater(mass, 0.8)  # 1.0 * (1 - 0.2)
            self.assertLess(mass, 1.2)  # 1.0 * (1 + 0.2)

        backend.close()

    @patch("upkie.envs.backends.pybullet_backend.pybullet_data")
    @patch("upkie.envs.backends.pybullet_backend.pybullet")
    @patch("upkie.envs.backends.pybullet_backend.upkie_description")
    def test_apply_external_forces_world_frame(
        self, mock_upkie_desc, mock_pybullet, mock_pybullet_data
    ):
        """Test applying external forces in world frame."""
        mock_pybullet.configure_mock(**self.pybullet_mock.__dict__)
        mock_pybullet_data.configure_mock(**self.pybullet_data_mock.__dict__)
        mock_upkie_desc.URDF_PATH = "/mock/urdf/path"

        # Mock required methods for get_spine_observation
        mock_pybullet.getBasePositionAndOrientation.return_value = (
            [0.5, 0.6, 0.7],  # base position
            [0.0, 0.0, 0.0, 1.0],  # base orientation
        )
        mock_pybullet.getBaseVelocity.return_value = (
            [0.0, 0.0, 0.0],  # linear velocity
            [0.0, 0.0, 0.0],  # angular velocity
        )
        mock_pybullet.getLinkState.return_value = [
            [1.0, 2.0, 3.0],  # world position
            [0.0, 0.0, 0.0, 1.0],  # world orientation
            [0.0, 0.0, 0.0],  # local inertial frame position
            [0.0, 0.0, 0.0, 1.0],  # local inertial frame orientation
            [0.0, 0.0, 0.0],  # world frame position of CoM
            [0.0, 0.0, 0.0, 1.0],  # world frame orientation
            [0.0, 0.0, 0.0],  # linear velocity
            [0.0, 0.0, 0.0],  # angular velocity
        ]

        backend = PyBulletBackend(dt=0.01, gui=False)

        # Define external forces
        external_forces = {
            "left_hip_link": ExternalForce(  # Valid link from mock setup
                [10.0, 20.0, 30.0], local=False  # world frame
            ),
            "imu": ExternalForce(  # Valid link from mock setup
                [5.0, 15.0, 25.0], local=False
            ),
        }

        # Apply external forces
        backend.set_external_forces(external_forces)

        # Step the simulation to trigger force application
        backend.step({})

        # Check that applyExternalForce was called correctly
        self.assertGreater(
            mock_pybullet.applyExternalForce.call_count,
            0,
            "applyExternalForce should have been called",
        )

        # Find calls for the imu link (index 6)
        imu_calls = [
            call
            for call in mock_pybullet.applyExternalForce.call_args_list
            if call[0][1] == 6  # imu link index
        ]

        # Verify imu call if it exists
        if imu_calls:
            imu_call = imu_calls[0]
            args, kwargs = imu_call
            robot_id, link_index, force, position, flags = args

            self.assertEqual(robot_id, backend._robot_id)
            self.assertEqual(link_index, 6)
            self.assertEqual(list(force), [5.0, 15.0, 25.0])
            self.assertEqual(flags, mock_pybullet.WORLD_FRAME)

        backend.close()

    @patch("upkie.envs.backends.pybullet_backend.pybullet_data")
    @patch("upkie.envs.backends.pybullet_backend.pybullet")
    @patch("upkie.envs.backends.pybullet_backend.upkie_description")
    def test_apply_external_forces_local_frame(
        self, mock_upkie_desc, mock_pybullet, mock_pybullet_data
    ):
        """Test applying external forces in local frame."""
        mock_pybullet.configure_mock(**self.pybullet_mock.__dict__)
        mock_pybullet_data.configure_mock(**self.pybullet_data_mock.__dict__)
        mock_upkie_desc.URDF_PATH = "/mock/urdf/path"

        # Mock required methods for get_spine_observation
        mock_pybullet.getBasePositionAndOrientation.return_value = (
            [0.0, 0.0, 0.6],
            [0.0, 0.0, 0.0, 1.0],
        )
        mock_pybullet.getBaseVelocity.return_value = (
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
        )
        mock_pybullet.getLinkState.return_value = [
            None,
            None,
            None,
            None,
            None,
            [0.0, 0.0, 0.0, 1.0],  # link orientation
            [0.0, 0.0, 0.0],  # linear velocity
            [0.0, 0.0, 0.0],  # angular velocity
        ]

        backend = PyBulletBackend(dt=0.01, gui=False)

        # Define external forces in local frame
        external_forces = {
            "base": ExternalForce([1.0, 2.0, 3.0], local=True)  # local frame
        }

        # Apply external forces
        backend.set_external_forces(external_forces)

        # Step the simulation
        backend.step({})

        # Check that applyExternalForce was called with local frame parameters
        base_calls = [
            call
            for call in mock_pybullet.applyExternalForce.call_args_list
            if call[0][1] == -1  # base link index
        ]

        self.assertGreater(
            len(base_calls),
            0,
            "applyExternalForce should have been called for base",
        )

        base_call = base_calls[0]
        args, kwargs = base_call
        robot_id, link_index, force, position, flags = args

        self.assertEqual(robot_id, backend._robot_id)
        self.assertEqual(link_index, -1)
        self.assertEqual(list(force), [1.0, 2.0, 3.0])
        self.assertEqual(list(position), [0.0, 0.0, 0.0])  # local origin
        self.assertEqual(flags, mock_pybullet.LINK_FRAME)

        backend.close()

    @patch("upkie.envs.backends.pybullet_backend.pybullet_data")
    @patch("upkie.envs.backends.pybullet_backend.pybullet")
    @patch("upkie.envs.backends.pybullet_backend.upkie_description")
    def test_apply_external_forces_invalid_force_shape(
        self, mock_upkie_desc, mock_pybullet, mock_pybullet_data
    ):
        """Test that invalid force shapes raise ValueError."""
        mock_pybullet.configure_mock(**self.pybullet_mock.__dict__)
        mock_pybullet_data.configure_mock(**self.pybullet_data_mock.__dict__)
        mock_upkie_desc.URDF_PATH = "/mock/urdf/path"

        backend = PyBulletBackend(dt=0.01, gui=False)

        # Test 2D force vector (should fail during ExternalForce creation)
        with self.assertRaises(ValueError) as context:
            ExternalForce([1.0, 2.0])  # Only 2 components

        self.assertIn("Force must be a 3D vector", str(context.exception))

        # Test 4D force vector (should fail during ExternalForce creation)
        with self.assertRaises(ValueError) as context:
            ExternalForce([1.0, 2.0, 3.0, 4.0])  # 4 components

        self.assertIn("Force must be a 3D vector", str(context.exception))

        backend.close()

    @patch("upkie.envs.backends.pybullet_backend.pybullet_data")
    @patch("upkie.envs.backends.pybullet_backend.pybullet")
    @patch("upkie.envs.backends.pybullet_backend.upkie_description")
    def test_apply_external_forces_unknown_link(
        self, mock_upkie_desc, mock_pybullet, mock_pybullet_data
    ):
        """Test behavior when applying forces to unknown links."""
        mock_pybullet.configure_mock(**self.pybullet_mock.__dict__)
        mock_pybullet_data.configure_mock(**self.pybullet_data_mock.__dict__)
        mock_upkie_desc.URDF_PATH = "/mock/urdf/path"

        # Mock required methods for get_spine_observation
        mock_pybullet.getBasePositionAndOrientation.return_value = (
            [0.0, 0.0, 0.6],
            [0.0, 0.0, 0.0, 1.0],
        )
        mock_pybullet.getBaseVelocity.return_value = (
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
        )
        mock_pybullet.getLinkState.return_value = [
            None,
            None,
            None,
            None,
            None,
            [0.0, 0.0, 0.0, 1.0],  # link orientation
            [0.0, 0.0, 0.0],  # linear velocity
            [0.0, 0.0, 0.0],  # angular velocity
        ]

        backend = PyBulletBackend(dt=0.01, gui=False)

        # Should raise UpkieRuntimeError for unknown link
        with self.assertRaises(UpkieRuntimeError) as context:
            backend.set_external_forces(
                {"unknown_link": ExternalForce([1.0, 2.0, 3.0])}
            )

        self.assertIn(
            "Robot does not have a link named 'unknown_link'",
            str(context.exception),
        )

        backend.close()

    @patch("upkie.envs.backends.pybullet_backend.pybullet_data")
    @patch("upkie.envs.backends.pybullet_backend.pybullet")
    @patch("upkie.envs.backends.pybullet_backend.upkie_description")
    def test_apply_external_forces_multiple_steps(
        self, mock_upkie_desc, mock_pybullet, mock_pybullet_data
    ):
        """Test that external forces persist across multiple steps."""
        mock_pybullet.configure_mock(**self.pybullet_mock.__dict__)
        mock_pybullet_data.configure_mock(**self.pybullet_data_mock.__dict__)
        mock_upkie_desc.URDF_PATH = "/mock/urdf/path"

        # Mock required methods for get_spine_observation
        mock_pybullet.getBasePositionAndOrientation.return_value = (
            [0.0, 0.0, 0.6],
            [0.0, 0.0, 0.0, 1.0],
        )
        mock_pybullet.getBaseVelocity.return_value = (
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
        )
        mock_pybullet.getLinkState.return_value = [
            None,
            None,
            None,
            None,
            None,
            [0.0, 0.0, 0.0, 1.0],  # link orientation
            [0.0, 0.0, 0.0],  # linear velocity
            [0.0, 0.0, 0.0],  # angular velocity
        ]

        backend = PyBulletBackend(dt=0.01, gui=False)

        # Apply external forces
        backend.set_external_forces(
            {"base": ExternalForce([0.0, 0.0, 10.0], local=False)}
        )

        # Step simulation multiple times
        for _ in range(3):
            backend.step({})

        # Should have called applyExternalForce in each step (once per substep)
        base_calls = [
            call
            for call in mock_pybullet.applyExternalForce.call_args_list
            if call[0][1] == -1  # base link
        ]

        # With default dt=0.01, nb_substeps = int(1000.0 * 0.01) = 10 substeps
        # per step, we have 3 steps * 10 substeps = 30 calls total
        expected_calls = 3 * 10  # 3 steps * 10 substeps per step
        self.assertEqual(
            len(base_calls),
            expected_calls,
            f"applyExternalForce should be called {expected_calls} times here",
        )

        # All calls should have the same force parameters
        for call in base_calls:
            args, kwargs = call
            robot_id, link_index, force, position, flags = args
            self.assertEqual(list(force), [0.0, 0.0, 10.0])

        backend.close()

    @patch("upkie.envs.backends.pybullet_backend.pybullet_data")
    @patch("upkie.envs.backends.pybullet_backend.pybullet")
    @patch("upkie.envs.backends.pybullet_backend.upkie_description")
    def test_apply_external_forces_update_forces(
        self, mock_upkie_desc, mock_pybullet, mock_pybullet_data
    ):
        """Test that external forces can be updated between steps."""
        mock_pybullet.configure_mock(**self.pybullet_mock.__dict__)
        mock_pybullet_data.configure_mock(**self.pybullet_data_mock.__dict__)
        mock_upkie_desc.URDF_PATH = "/mock/urdf/path"

        # Mock required methods for get_spine_observation
        mock_pybullet.getBasePositionAndOrientation.return_value = (
            [0.0, 0.0, 0.6],
            [0.0, 0.0, 0.0, 1.0],
        )
        mock_pybullet.getBaseVelocity.return_value = (
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
        )
        mock_pybullet.getLinkState.return_value = [
            None,
            None,
            None,
            None,
            None,
            [0.0, 0.0, 0.0, 1.0],  # link orientation
            [0.0, 0.0, 0.0],  # linear velocity
            [0.0, 0.0, 0.0],  # angular velocity
        ]

        backend = PyBulletBackend(dt=0.01, gui=False)

        # Apply initial forces
        backend.set_external_forces(
            {"base": ExternalForce([1.0, 0.0, 0.0], local=False)}
        )
        backend.step({})

        # Update forces
        backend.set_external_forces(
            {"base": ExternalForce([0.0, 2.0, 0.0], local=False)}
        )
        backend.step({})

        # Check that forces were applied correctly in both steps
        base_calls = [
            call
            for call in mock_pybullet.applyExternalForce.call_args_list
            if call[0][1] == -1  # base link
        ]

        # With 2 steps * 10 substeps per step = 20 calls total
        expected_calls = 2 * 10
        self.assertEqual(len(base_calls), expected_calls)

        # First 10 calls should have force [1.0, 0.0, 0.0] (first step)
        for i in range(10):
            first_force = list(base_calls[i][0][2])
            self.assertEqual(first_force, [1.0, 0.0, 0.0])

        # Next 10 calls should have force [0.0, 2.0, 0.0] (second step)
        for i in range(10, 20):
            second_force = list(base_calls[i][0][2])
            self.assertEqual(second_force, [0.0, 2.0, 0.0])

        backend.close()

    @patch("upkie.envs.backends.pybullet_backend.pybullet_data")
    @patch("upkie.envs.backends.pybullet_backend.pybullet")
    @patch("upkie.envs.backends.pybullet_backend.upkie_description")
    def test_get_contact_points_basic(
        self, mock_upkie_desc, mock_pybullet, mock_pybullet_data
    ):
        """Test basic functionality of get_contact_points method."""
        mock_pybullet.configure_mock(**self.pybullet_mock.__dict__)
        mock_pybullet_data.configure_mock(**self.pybullet_data_mock.__dict__)
        mock_upkie_desc.URDF_PATH = "/mock/urdf/path"

        # Mock contact points data
        mock_contact_data = [
            (
                0,  # contact_flag
                1,  # body_unique_id_a (robot)
                0,  # body_unique_id_b (ground plane)
                5,  # link_index_a (right wheel)
                -1,  # link_index_b (base of plane)
                [0.1, 0.2, 0.3],  # position_on_a
                [0.1, 0.2, 0.0],  # position_on_b
                [0.0, 0.0, 1.0],  # contact_normal_on_b
                -0.001,  # contact_distance (negative = penetration)
                100.0,  # normal_force
                0.0,  # lateral_friction_1
                [1.0, 0.0, 0.0],  # lateral_friction_dir_1
                0.0,  # lateral_friction_2
                [0.0, 1.0, 0.0],  # lateral_friction_dir_2
            ),
            (
                0,  # contact_flag
                1,  # body_unique_id_a (robot)
                0,  # body_unique_id_b (ground plane)
                2,  # link_index_a (left wheel)
                -1,  # link_index_b (base of plane)
                [-0.1, 0.2, 0.3],  # position_on_a
                [-0.1, 0.2, 0.0],  # position_on_b
                [0.0, 0.0, 1.0],  # contact_normal_on_b
                -0.002,  # contact_distance
                95.0,  # normal_force
                0.0,  # lateral_friction_1
                [1.0, 0.0, 0.0],  # lateral_friction_dir_1
                0.0,  # lateral_friction_2
                [0.0, 1.0, 0.0],  # lateral_friction_dir_2
            ),
        ]

        mock_pybullet.getContactPoints.return_value = mock_contact_data

        backend = PyBulletBackend(dt=0.01, gui=False)

        # Test getting contact points with default parameters (robot vs all)
        contact_points = backend.get_contact_points()

        # Should have called getContactPoints with robot body ID
        mock_pybullet.getContactPoints.assert_called_with(
            bodyA=backend._robot_id, bodyB=-1, linkIndexA=-2, linkIndexB=-2
        )

        # Should return 2 contact points
        self.assertEqual(len(contact_points), 2)

        # Check first contact point structure
        contact1 = contact_points[0]
        self.assertEqual(contact1["contact_flag"], 0)
        self.assertEqual(contact1["body_unique_id_a"], 1)
        self.assertEqual(contact1["body_unique_id_b"], 0)
        self.assertEqual(contact1["link_index_a"], 5)
        self.assertEqual(contact1["link_index_b"], -1)
        self.assertEqual(contact1["position_on_a"], [0.1, 0.2, 0.3])
        self.assertEqual(contact1["position_on_b"], [0.1, 0.2, 0.0])
        self.assertEqual(contact1["contact_normal_on_b"], [0.0, 0.0, 1.0])
        self.assertEqual(contact1["contact_distance"], -0.001)
        self.assertEqual(contact1["normal_force"], 100.0)
        self.assertEqual(contact1["lateral_friction_1"], 0.0)
        self.assertEqual(contact1["lateral_friction_dir_1"], [1.0, 0.0, 0.0])
        self.assertEqual(contact1["lateral_friction_2"], 0.0)
        self.assertEqual(contact1["lateral_friction_dir_2"], [0.0, 1.0, 0.0])

        # Check second contact point
        contact2 = contact_points[1]
        self.assertEqual(contact2["link_index_a"], 2)
        self.assertEqual(contact2["normal_force"], 95.0)
        self.assertEqual(contact2["contact_distance"], -0.002)

        backend.close()

    @patch("upkie.envs.backends.pybullet_backend.pybullet_data")
    @patch("upkie.envs.backends.pybullet_backend.pybullet")
    @patch("upkie.envs.backends.pybullet_backend.upkie_description")
    def test_get_contact_points_no_contacts(
        self, mock_upkie_desc, mock_pybullet, mock_pybullet_data
    ):
        """Test get_contact_points when no contacts exist."""
        mock_pybullet.configure_mock(**self.pybullet_mock.__dict__)
        mock_pybullet_data.configure_mock(**self.pybullet_data_mock.__dict__)
        mock_upkie_desc.URDF_PATH = "/mock/urdf/path"

        # Mock empty contact points
        mock_pybullet.getContactPoints.return_value = []

        backend = PyBulletBackend(dt=0.01, gui=False)

        # Test with no contacts
        contact_points = backend.get_contact_points()

        self.assertEqual(len(contact_points), 0)
        self.assertIsInstance(contact_points, list)

        backend.close()

    @patch("upkie.envs.backends.pybullet_backend.pybullet_data")
    @patch("upkie.envs.backends.pybullet_backend.pybullet")
    @patch("upkie.envs.backends.pybullet_backend.upkie_description")
    def test_get_contact_points_robot_only(
        self, mock_upkie_desc, mock_pybullet, mock_pybullet_data
    ):
        """Test get_contact_points always uses robot ID for body_a."""
        mock_pybullet.configure_mock(**self.pybullet_mock.__dict__)
        mock_pybullet_data.configure_mock(**self.pybullet_data_mock.__dict__)
        mock_upkie_desc.URDF_PATH = "/mock/urdf/path"

        mock_pybullet.getContactPoints.return_value = []

        backend = PyBulletBackend(dt=0.01, gui=False)
        robot_id = backend._robot_id

        # Test that function gets all contacts for the robot
        backend.get_contact_points()
        mock_pybullet.getContactPoints.assert_called_with(
            bodyA=robot_id, bodyB=-1, linkIndexA=-2, linkIndexB=-2
        )

        backend.close()

    @patch("upkie.envs.backends.pybullet_backend.pybullet_data")
    @patch("upkie.envs.backends.pybullet_backend.pybullet")
    @patch("upkie.envs.backends.pybullet_backend.upkie_description")
    def test_get_contact_points_with_link_name(
        self, mock_upkie_desc, mock_pybullet, mock_pybullet_data
    ):
        """Test get_contact_points with link_name filtering."""
        mock_pybullet.configure_mock(**self.pybullet_mock.__dict__)
        mock_pybullet_data.configure_mock(**self.pybullet_data_mock.__dict__)
        mock_upkie_desc.URDF_PATH = "/mock/urdf/path"

        # Mock contact data with different links
        mock_contact_data = [
            (
                0,
                1,
                0,
                5,
                -1,
                [0.1, 0.2, 0.3],
                [0.1, 0.2, 0.0],
                [0.0, 0.0, 1.0],
                -0.001,
                100.0,
                0.0,
                [1.0, 0.0, 0.0],
                0.0,
                [0.0, 1.0, 0.0],
            ),
            (
                0,
                1,
                0,
                2,
                -1,
                [-0.1, 0.2, 0.3],
                [-0.1, 0.2, 0.0],
                [0.0, 0.0, 1.0],
                -0.002,
                95.0,
                0.0,
                [1.0, 0.0, 0.0],
                0.0,
                [0.0, 1.0, 0.0],
            ),
            (
                0,
                1,
                0,
                7,
                -1,
                [0.0, 0.0, 0.1],
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 1.0],
                -0.001,
                50.0,
                0.0,
                [1.0, 0.0, 0.0],
                0.0,
                [0.0, 1.0, 0.0],
            ),
        ]

        mock_pybullet.getContactPoints.return_value = mock_contact_data

        # Mock joint info for get_link_index
        mock_joint_info_list = [
            (
                0,
                b"joint1",
                1,
                -1,
                0,
                0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                b"link1",
            ),
            (
                1,
                b"joint2",
                1,
                -1,
                0,
                0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                b"left_wheel_tire",
            ),
            (
                2,
                b"joint3",
                1,
                -1,
                0,
                0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                b"link3",
            ),
            (
                3,
                b"imu_joint",
                1,
                -1,
                0,
                0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                b"imu",
            ),
        ]

        def mock_get_joint_info(robot_id, joint_index):
            if joint_index < len(mock_joint_info_list):
                return mock_joint_info_list[joint_index]
            raise IndexError("Joint index out of range")

        mock_pybullet.getJointInfo.side_effect = mock_get_joint_info
        mock_pybullet.getNumJoints.return_value = len(mock_joint_info_list)

        backend = PyBulletBackend(dt=0.01, gui=False)

        # Set up different return values for different calls
        def mock_get_contact_points_side_effect(*args, **kwargs):
            # Check if linkIndexA is specified
            if "linkIndexA" in kwargs:
                link_index = kwargs["linkIndexA"]
                if link_index == 1:  # left_wheel_tire
                    # Empty list as no contact for this link in test data
                    return []
                elif link_index == 2:  # link3
                    # Return only contacts for this link
                    return [
                        (
                            0,
                            1,
                            0,
                            2,
                            -1,
                            [-0.1, 0.2, 0.3],
                            [-0.1, 0.2, 0.0],
                            [0.0, 0.0, 1.0],
                            -0.002,
                            95.0,
                            0.0,
                            [1.0, 0.0, 0.0],
                            0.0,
                            [0.0, 1.0, 0.0],
                        )
                    ]
            # Default return all contacts
            return mock_contact_data

        mock_pybullet.getContactPoints.side_effect = (
            mock_get_contact_points_side_effect
        )

        # Test filtering by specific link name that has no contacts
        contact_points = backend.get_contact_points(
            link_name="left_wheel_tire"
        )
        self.assertEqual(len(contact_points), 0)

        # Verify the correct API call was made
        mock_pybullet.getContactPoints.assert_called_with(
            bodyA=backend._robot_id, bodyB=-1, linkIndexA=1, linkIndexB=-2
        )

        # Test with a link that has contacts (link index 2)
        contact_points = backend.get_contact_points(link_name="link3")
        self.assertEqual(len(contact_points), 1)
        self.assertEqual(contact_points[0]["link_index_a"], 2)

        # Verify the correct API call was made
        mock_pybullet.getContactPoints.assert_called_with(
            bodyA=backend._robot_id, bodyB=-1, linkIndexA=2, linkIndexB=-2
        )

        backend.close()

    @patch("upkie.envs.backends.pybullet_backend.pybullet_data")
    @patch("upkie.envs.backends.pybullet_backend.pybullet")
    @patch("upkie.envs.backends.pybullet_backend.upkie_description")
    def test_get_contact_points_invalid_link_name(
        self, mock_upkie_desc, mock_pybullet, mock_pybullet_data
    ):
        """Test get_contact_points with invalid link name."""
        mock_pybullet.configure_mock(**self.pybullet_mock.__dict__)
        mock_pybullet_data.configure_mock(**self.pybullet_data_mock.__dict__)
        mock_upkie_desc.URDF_PATH = "/mock/urdf/path"

        # Mock joint info with IMU link
        mock_joint_info_list = [
            (
                0,
                b"imu_joint",
                1,
                -1,
                0,
                0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                b"imu",
            ),
        ]

        def mock_get_joint_info(robot_id, joint_index):
            if joint_index < len(mock_joint_info_list):
                return mock_joint_info_list[joint_index]
            raise IndexError("Joint index out of range")

        mock_pybullet.getJointInfo.side_effect = mock_get_joint_info
        mock_pybullet.getNumJoints.return_value = len(mock_joint_info_list)
        mock_pybullet.getContactPoints.return_value = []

        backend = PyBulletBackend(dt=0.01, gui=False)

        # Test with invalid link name
        contact_points = backend.get_contact_points(
            link_name="nonexistent_link"
        )
        self.assertEqual(len(contact_points), 0)

        backend.close()

    @patch("upkie.envs.backends.pybullet_backend.pybullet_data")
    @patch("upkie.envs.backends.pybullet_backend.pybullet")
    @patch("upkie.envs.backends.pybullet_backend.upkie_description")
    def test_get_contact_points_base_link(
        self, mock_upkie_desc, mock_pybullet, mock_pybullet_data
    ):
        """Test get_contact_points with base link."""
        mock_pybullet.configure_mock(**self.pybullet_mock.__dict__)
        mock_pybullet_data.configure_mock(**self.pybullet_data_mock.__dict__)
        mock_upkie_desc.URDF_PATH = "/mock/urdf/path"

        # Mock joint info with IMU link
        mock_joint_info_list = [
            (
                0,
                b"imu_joint",
                1,
                -1,
                0,
                0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                b"imu",
            ),
        ]

        def mock_get_joint_info(robot_id, joint_index):
            if joint_index < len(mock_joint_info_list):
                return mock_joint_info_list[joint_index]
            raise IndexError("Joint index out of range")

        mock_pybullet.getJointInfo.side_effect = mock_get_joint_info
        mock_pybullet.getNumJoints.return_value = len(mock_joint_info_list)

        # Mock contact points side effect for different API calls
        def mock_get_contact_points_side_effect(*args, **kwargs):
            # Check if linkIndexA is -1 (base link)
            if "linkIndexA" in kwargs and kwargs["linkIndexA"] == -1:
                # Return only base link contact
                return [
                    (
                        0,
                        1,
                        0,
                        -1,
                        -1,
                        [0.0, 0.0, 0.1],
                        [0.0, 0.0, 0.0],
                        [0.0, 0.0, 1.0],
                        -0.001,
                        50.0,
                        0.0,
                        [1.0, 0.0, 0.0],
                        0.0,
                        [0.0, 1.0, 0.0],
                    )
                ]
            # Default return all contacts
            return [
                (
                    0,
                    1,
                    0,
                    -1,
                    -1,
                    [0.0, 0.0, 0.1],
                    [0.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0],
                    -0.001,
                    50.0,
                    0.0,
                    [1.0, 0.0, 0.0],
                    0.0,
                    [0.0, 1.0, 0.0],
                ),
                (
                    0,
                    1,
                    0,
                    2,
                    -1,
                    [-0.1, 0.2, 0.3],
                    [-0.1, 0.2, 0.0],
                    [0.0, 0.0, 1.0],
                    -0.002,
                    95.0,
                    0.0,
                    [1.0, 0.0, 0.0],
                    0.0,
                    [0.0, 1.0, 0.0],
                ),
            ]

        mock_pybullet.getContactPoints.side_effect = (
            mock_get_contact_points_side_effect
        )

        backend = PyBulletBackend(dt=0.01, gui=False)

        # Test filtering by base link
        contact_points = backend.get_contact_points(link_name="base")

        # Should only return contacts for base link (index -1)
        self.assertEqual(len(contact_points), 1)
        self.assertEqual(contact_points[0]["link_index_a"], -1)

        # Verify the correct API call was made
        mock_pybullet.getContactPoints.assert_called_with(
            bodyA=backend._robot_id, bodyB=-1, linkIndexA=-1, linkIndexB=-2
        )

        backend.close()

    @patch("upkie.envs.backends.pybullet_backend.pybullet_data")
    @patch("upkie.envs.backends.pybullet_backend.pybullet")
    @patch("upkie.envs.backends.pybullet_backend.upkie_description")
    def test_apply_external_forces_with_external_force_class(
        self, mock_upkie_desc, mock_pybullet, mock_pybullet_data
    ):
        """Test applying external forces using ExternalForce class."""
        mock_pybullet.configure_mock(**self.pybullet_mock.__dict__)
        mock_pybullet_data.configure_mock(**self.pybullet_data_mock.__dict__)
        mock_upkie_desc.URDF_PATH = "/mock/urdf/path"

        # Mock required methods for get_spine_observation
        mock_pybullet.getBasePositionAndOrientation.return_value = (
            [0.0, 0.0, 0.6],
            [0.0, 0.0, 0.0, 1.0],
        )
        mock_pybullet.getBaseVelocity.return_value = (
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
        )
        mock_pybullet.getLinkState.return_value = [
            None,
            None,
            None,
            None,
            None,
            [0.0, 0.0, 0.0, 1.0],  # link orientation
            [0.0, 0.0, 0.0],  # linear velocity
            [0.0, 0.0, 0.0],  # angular velocity
        ]

        backend = PyBulletBackend(dt=0.01, gui=False)

        # Test with ExternalForce instances
        external_forces = {
            "base": ExternalForce([5.0, 10.0, 15.0], local=True),
            "imu": ExternalForce(force=np.array([1.0, 2.0, 3.0]), local=False),
        }

        # Apply external forces
        backend.set_external_forces(external_forces)

        # Step the simulation to trigger force application
        backend.step({})

        # Check that applyExternalForce was called
        self.assertGreater(
            mock_pybullet.applyExternalForce.call_count,
            0,
            "applyExternalForce should have been called",
        )

        # Find calls for base and imu links
        base_calls = [
            call
            for call in mock_pybullet.applyExternalForce.call_args_list
            if call[0][1] == -1  # base link index
        ]
        imu_calls = [
            call
            for call in mock_pybullet.applyExternalForce.call_args_list
            if call[0][1] == 6  # imu link index
        ]

        # Verify base call (local frame)
        if base_calls:
            base_call = base_calls[0]
            args, kwargs = base_call
            robot_id, link_index, force, position, flags = args

            self.assertEqual(robot_id, backend._robot_id)
            self.assertEqual(link_index, -1)
            self.assertEqual(list(force), [5.0, 10.0, 15.0])
            self.assertEqual(list(position), [0.0, 0.0, 0.0])  # local origin
            self.assertEqual(flags, mock_pybullet.LINK_FRAME)

        # Verify imu call (world frame)
        if imu_calls:
            imu_call = imu_calls[0]
            args, kwargs = imu_call
            robot_id, link_index, force, position, flags = args

            self.assertEqual(robot_id, backend._robot_id)
            self.assertEqual(link_index, 6)
            self.assertEqual(list(force), [1.0, 2.0, 3.0])
            self.assertEqual(flags, mock_pybullet.WORLD_FRAME)

        backend.close()

    @patch("upkie.envs.backends.pybullet_backend.pybullet_data")
    @patch("upkie.envs.backends.pybullet_backend.pybullet")
    @patch("upkie.envs.backends.pybullet_backend.upkie_description")
    def test_apply_external_forces_mixed_formats(
        self, mock_upkie_desc, mock_pybullet, mock_pybullet_data
    ):
        """Test mixing ExternalForce instances and dictionary formats."""
        mock_pybullet.configure_mock(**self.pybullet_mock.__dict__)
        mock_pybullet_data.configure_mock(**self.pybullet_data_mock.__dict__)
        mock_upkie_desc.URDF_PATH = "/mock/urdf/path"

        # Mock required methods for get_spine_observation
        mock_pybullet.getBasePositionAndOrientation.return_value = (
            [0.0, 0.0, 0.6],
            [0.0, 0.0, 0.0, 1.0],
        )
        mock_pybullet.getBaseVelocity.return_value = (
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
        )
        mock_pybullet.getLinkState.return_value = [
            None,
            None,
            None,
            None,
            None,
            [0.0, 0.0, 0.0, 1.0],  # link orientation
            [0.0, 0.0, 0.0],  # linear velocity
            [0.0, 0.0, 0.0],  # angular velocity
        ]

        backend = PyBulletBackend(dt=0.01, gui=False)

        # Test with multiple ExternalForce instances
        external_forces = {
            "base": ExternalForce(
                [1.0, 0.0, 0.0], local=True
            ),  # ExternalForce
            "imu": ExternalForce(
                [0.0, 1.0, 0.0], local=False
            ),  # ExternalForce
        }

        # Should work without error
        backend.set_external_forces(external_forces)
        backend.step({})

        # Verify both formats were processed
        self.assertGreater(
            mock_pybullet.applyExternalForce.call_count,
            0,
            "Forces should be applied for both formats",
        )

        backend.close()


if __name__ == "__main__":
    unittest.main()
