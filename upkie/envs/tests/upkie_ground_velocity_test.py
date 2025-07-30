#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

"""Test UpkieGroundVelocity."""

import unittest
from multiprocessing.shared_memory import SharedMemory

import numpy as np

from upkie.envs import UpkieSpineServos
from upkie.envs.tests.mock_spine import MockSpine
from upkie.envs.upkie_ground_velocity import UpkieGroundVelocity


class TestUpkieGroundVelocity(unittest.TestCase):
    def setUp(self):
        shared_memory = SharedMemory(name=None, size=42, create=True)
        servos_env = UpkieSpineServos(
            frequency=100.0,
            shm_name=shared_memory._name,
        )
        servos_env._spine = MockSpine()
        ground_velocity_env = UpkieGroundVelocity(
            servos_env,
            fall_pitch=1.0,
            max_ground_velocity=1.0,
        )
        shared_memory.close()
        self.servos_env = servos_env
        self.env = ground_velocity_env

    def test_reset(self):
        observation, info = self.env.reset()
        spine_observation = info["spine_observation"]
        self.assertAlmostEqual(
            observation[1], spine_observation["wheel_odometry"]["position"]
        )
        self.assertGreaterEqual(spine_observation["number"], 1)

    def test_reward(self):
        observation, _ = self.env.reset()
        action = np.zeros(self.env.action_space.shape)
        observation, reward, terminated, truncated, _ = self.env.step(action)
        self.assertNotEqual(reward, 0.0)  # non-zero base velocity

        spine_observation = self.servos_env._spine.observation
        base_orientation = spine_observation["base_orientation"]
        base_orientation["pitch"] = 0.0
        base_orientation["angular_velocity"] = [0.0, 0.0, 0.0]
        base_orientation["linear_velocity"] = [0.0, 0.0, 0.0]
        observation, reward, terminated, truncated, _ = self.env.step(action)
        self.assertAlmostEqual(reward, 1.0)  # ideal base state

    def test_check_env(self):
        try:
            from stable_baselines3.common.env_checker import check_env

            check_env(self.env)
        except ImportError:
            pass

    def test_maximum_torques(self):
        _, _ = self.env.reset()
        action = np.zeros(self.env.action_space.shape)
        _, _, _, _, _ = self.env.step(action)
        spine_action = self.servos_env._spine.action
        servo_action = spine_action["servo"]
        model = self.env.get_wrapper_attr("model")
        for joint in model.upper_leg_joints:
            maximum_torque = servo_action[joint.name]["maximum_torque"]
            self.assertLess(maximum_torque, 20.0)
        for wheel in model.wheel_joints:
            maximum_torque = servo_action[wheel.name]["maximum_torque"]
            self.assertLess(maximum_torque, 2.0)

    def test_max_ground_velocity(self):
        _, _ = self.env.reset()
        action = np.full(self.env.action_space.shape, 1111.1)
        _, _, _, _, _ = self.env.step(action)
        spine_action = self.servos_env._spine.action
        servo_action = spine_action["servo"]
        max_ground_velocity = self.env.action_space.high[0]
        model = self.env.get_wrapper_attr("model")
        for wheel in model.wheel_joints:
            wheel_velocity = servo_action[wheel.name]["velocity"]  # rad/s
            ground_velocity = np.abs(wheel_velocity * self.env.wheel_radius)
            self.assertLess(ground_velocity, max_ground_velocity + 1e-10)


if __name__ == "__main__":
    unittest.main()
