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


if __name__ == "__main__":
    unittest.main()
