#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2025 Inria

"""Tests for PyBullet backend friction functionality."""

import unittest
from unittest.mock import MagicMock, patch

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


if __name__ == "__main__":
    unittest.main()
