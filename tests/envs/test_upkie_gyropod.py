#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

"""Test UpkieGyropod wrapper."""

import unittest
from multiprocessing.shared_memory import SharedMemory

import numpy as np

from upkie.envs.backends import SpineBackend
from upkie.envs.testing import MockSpine
from upkie.envs.upkie_gyropod import UpkieGyropod
from upkie.envs.upkie_servos import UpkieServos


class GyropodTestCase(unittest.TestCase):
    def setUp(self):
        shared_memory = SharedMemory(name=None, size=42, create=True)
        self.backend = SpineBackend(shm_name=shared_memory._name)
        servos_env = UpkieServos(
            backend=self.backend,
            frequency=100.0,
        )
        self.backend._spine = MockSpine()
        gyropod = UpkieGyropod(
            servos_env,
            fall_pitch=1.0,
            max_ground_velocity=1.0,
            max_yaw_velocity=1.0,
        )
        shared_memory.close()
        self.servos_env = servos_env
        self.env = gyropod

    def test_observation_shape(self):
        observation, _ = self.env.reset()
        self.assertEqual(observation.shape, (6,))

    def test_action_shape(self):
        self.assertEqual(self.env.action_space.shape, (2,))

    def test_reset(self):
        observation, info = self.env.reset()
        spine_observation = info["spine_observation"]
        self.assertAlmostEqual(
            observation[0], spine_observation["wheel_odometry"]["position"]
        )
        self.assertAlmostEqual(
            observation[1], spine_observation["base_orientation"]["pitch"]
        )
        # Yaw angle should be zero after reset
        self.assertAlmostEqual(observation[2], 0.0)
        # Yaw velocity should be zero after reset
        self.assertAlmostEqual(observation[5], 0.0)
        self.assertGreaterEqual(spine_observation["number"], 1)

    def test_yaw_integration(self):
        self.env.reset()
        yaw_velocity = 0.5  # rad/s
        action = np.array([0.0, yaw_velocity], dtype=np.float32)
        observation, _, _, _, _ = self.env.step(action)
        dt = self.env.unwrapped.dt
        expected_yaw = yaw_velocity * dt
        self.assertAlmostEqual(observation[2], expected_yaw, places=5)
        self.assertAlmostEqual(observation[5], yaw_velocity, places=5)

    def test_yaw_resets_to_zero(self):
        self.env.reset()
        action = np.array([0.0, 1.0], dtype=np.float32)
        self.env.step(action)
        observation, _ = self.env.reset()
        self.assertAlmostEqual(observation[2], 0.0)

    def test_yaw_produces_differential_wheel_velocity(self):
        self.env.reset()
        # Pure yaw action (no sagittal velocity)
        action = np.array([0.0, 0.5], dtype=np.float32)
        self.env.step(action)
        spine_action = self.backend._spine.action
        servo_action = spine_action["servo"]
        left_vel = servo_action["left_wheel"]["velocity"]
        right_vel = servo_action["right_wheel"]["velocity"]
        # Both wheels should have the same sign for yaw rotation
        self.assertAlmostEqual(left_vel, right_vel)
        # And they should be non-zero
        self.assertNotAlmostEqual(left_vel, 0.0)

    def test_check_env(self):
        try:
            from stable_baselines3.common.env_checker import check_env

            check_env(self.env)
        except ImportError:
            pass

    def test_maximum_torques(self):
        self.env.reset()
        action = np.zeros(self.env.action_space.shape)
        self.env.step(action)
        spine_action = self.backend._spine.action
        servo_action = spine_action["servo"]
        model = self.env.get_wrapper_attr("model")
        for joint in model.upper_leg_joints:
            maximum_torque = servo_action[joint.name]["maximum_torque"]
            self.assertLess(maximum_torque, 20.0)
        for wheel in model.wheel_joints:
            maximum_torque = servo_action[wheel.name]["maximum_torque"]
            self.assertLess(maximum_torque, 2.0)

    def test_leg_gain_scale(self):
        self.env.reset()
        self.env.leg_gain_scale = 2.5
        action = np.zeros(self.env.action_space.shape)
        self.env.step(action)
        spine_action = self.backend._spine.action
        servo_action = spine_action["servo"]
        model = self.env.get_wrapper_attr("model")
        for joint in model.upper_leg_joints:
            self.assertAlmostEqual(servo_action[joint.name]["kp_scale"], 2.5)
            self.assertAlmostEqual(servo_action[joint.name]["kd_scale"], 2.5)

    def test_dtype_consistency(self):
        self.assertEqual(self.env.action_space.dtype, np.float32)
        self.assertEqual(self.env.observation_space.dtype, np.float32)

        observation, _ = self.env.reset()
        self.assertEqual(observation.dtype, np.float32)

        action = np.zeros(self.env.action_space.shape, dtype=np.float32)
        observation, _, _, _, _ = self.env.step(action)
        self.assertEqual(observation.dtype, np.float32)


if __name__ == "__main__":
    unittest.main()
