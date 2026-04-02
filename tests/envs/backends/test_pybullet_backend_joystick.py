#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

"""Tests covering joystick usage in the PyBullet backend."""

import unittest
from unittest.mock import MagicMock, patch

from upkie.envs.backends.pybullet_backend import PyBulletBackend
from upkie.exceptions import UpkieRuntimeError


class PyBulletBackendJoystickTestCase(unittest.TestCase):
    def setUp(self):
        self.pybullet_mock = MagicMock()
        self.pybullet_data_mock = MagicMock()
        self.pybullet_data_mock.getDataPath.return_value = "/mock/path"

        joint_infos = [
            (0, b"left_hip", b"left_hip_link"),
            (1, b"left_knee", b"left_knee_link"),
            (2, b"left_wheel", b"left_wheel_link"),
            (3, b"left_wheel_tire_joint", b"left_wheel_tire"),
            (4, b"right_hip", b"right_hip_link"),
            (5, b"right_knee", b"right_knee_link"),
            (6, b"right_wheel", b"right_wheel_link"),
            (7, b"right_wheel_tire_joint", b"right_wheel_tire"),
            (8, b"imu_joint", b"imu"),
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
        self.pybullet_mock.getJointState.return_value = (0.0, 0.0, 0.0, 0.0)

        # Observation mocks needed for get_spine_observation()
        self.pybullet_mock.getBasePositionAndOrientation.return_value = (
            [0.0, 0.0, 0.6],
            [0.0, 0.0, 0.0, 1.0],
        )
        self.pybullet_mock.getBaseVelocity.return_value = (
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
        )
        self.pybullet_mock.getLinkState.return_value = [
            None,
            None,
            None,
            None,
            None,
            [0.0, 0.0, 0.0, 1.0],  # orientation
            [0.0, 0.0, 0.0],  # linear velocity
            [0.0, 0.0, 0.0],  # angular velocity
        ]

    @patch("upkie.envs.backends.pybullet_backend.pybullet_data")
    @patch("upkie.envs.backends.pybullet_backend.pybullet")
    @patch("upkie.envs.backends.pybullet_backend.Model")
    def test_no_joystick_when_device_absent(
        self, mock_model, mock_pybullet, mock_pybullet_data
    ):
        """Check that joystick is None when the device path does not exist."""
        mock_pybullet.configure_mock(**self.pybullet_mock.__dict__)
        mock_pybullet_data.configure_mock(**self.pybullet_data_mock.__dict__)
        mock_model.return_value.urdf_path = "/mock/urdf/path"
        mock_model.JOINT_NAMES = (
            "left_hip", "left_knee", "left_wheel",
            "right_hip", "right_knee", "right_wheel",
        )

        backend = PyBulletBackend(
            dt=0.01,
            gui=False,
            js_path="/dev/input/nonexistent_joystick",
        )
        self.assertIsNone(backend.joystick)
        backend.close()

    @patch("upkie.envs.backends.pybullet_backend.Joystick")
    @patch("upkie.envs.backends.pybullet_backend.Path")
    @patch("upkie.envs.backends.pybullet_backend.pybullet_data")
    @patch("upkie.envs.backends.pybullet_backend.pybullet")
    @patch("upkie.envs.backends.pybullet_backend.Model")
    def test_b_button_raises_in_step(
        self,
        mock_model,
        mock_pybullet,
        mock_pybullet_data,
        mock_path_cls,
        mock_joystick_cls,
    ):
        """Check that step() calls self.joystick.write when it exists.

        In this test, write raises an UpkieRuntimeError, as happens when the B
        button is pressed."""
        mock_pybullet.configure_mock(**self.pybullet_mock.__dict__)
        mock_pybullet_data.configure_mock(**self.pybullet_data_mock.__dict__)
        mock_model.return_value.urdf_path = "/mock/urdf/path"
        mock_model.JOINT_NAMES = (
            "left_hip", "left_knee", "left_wheel",
            "right_hip", "right_knee", "right_wheel",
        )

        # Make Path(js_path).exists() return True so the Joystick is created
        mock_path_instance = MagicMock()
        mock_path_instance.exists.return_value = True
        mock_path_cls.return_value = mock_path_instance

        # Make the joystick's write() raise UpkieRuntimeError (B pressed)
        mock_joystick_instance = MagicMock()
        mock_joystick_instance.write.side_effect = UpkieRuntimeError(
            "Stop button pressed"
        )
        mock_joystick_cls.return_value = mock_joystick_instance

        backend = PyBulletBackend(dt=0.01, gui=False, js_path="/dev/input/js0")
        self.assertIsNotNone(backend.joystick)

        with self.assertRaises(UpkieRuntimeError):
            backend.step(action={})

        backend.close()


if __name__ == "__main__":
    unittest.main()
