# SPDX-License-Identifier: Apache-2.0

"""Test UpkieGyropod wrapper."""

import unittest
from multiprocessing.shared_memory import SharedMemory

import numpy as np

from upkie.envs.backends import SpineBackend
from upkie.envs.testing import MockSpine
from upkie.envs.upkie_gyropod import UpkieGyropod
from upkie.envs.upkie_servos import UpkieServos
from upkie.exceptions import UpkieException


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
        self.env.set_leg_gain_scale(2.5)
        self.assertAlmostEqual(self.env.leg_gain_scale, 2.5)
        action = np.zeros(self.env.action_space.shape)
        self.env.step(action)
        spine_action = self.backend._spine.action
        servo_action = spine_action["servo"]
        model = self.env.get_wrapper_attr("model")
        for joint in model.upper_leg_joints:
            self.assertAlmostEqual(servo_action[joint.name]["kp_scale"], 2.5)
            self.assertAlmostEqual(servo_action[joint.name]["kd_scale"], 2.5)

    def leg_targets_after(self, env: UpkieGyropod, nb_steps: int) -> dict:
        """Reset an environment and step it with zero actions.

        Args:
            env: Gyropod environment around the mock spine.
            nb_steps: Number of steps.

        Returns:
            Position targets of the upper leg joints in the last spine action.
        """
        env.reset()
        action = np.zeros(env.action_space.shape)
        for _ in range(nb_steps):
            env.step(action)
        servo_action = self.backend._spine.action["servo"]
        joints = ("left_hip", "left_knee", "right_hip", "right_knee")
        return {joint: servo_action[joint]["position"] for joint in joints}

    def test_default_crouching_is_straight(self):
        self.assertEqual(self.env.crouching_height, 0.0)
        targets = self.leg_targets_after(self.env, nb_steps=1000)
        for joint, target in targets.items():
            self.assertAlmostEqual(target, 0.0, msg=joint)

    def test_crouching_posture(self):
        env = UpkieGyropod(self.servos_env, crouching_height=0.05)
        self.assertEqual(env.crouching_height, 0.05)
        expected = self.servos_env.model.compute_crouching_posture(0.05)
        targets = self.leg_targets_after(env, nb_steps=1000)  # 10 s
        for joint, target in targets.items():
            self.assertAlmostEqual(target, expected[joint], places=4)
        self.assertGreater(targets["left_hip"], 0.0)  # knees forward
        self.assertLess(targets["left_knee"], 0.0)
        self.assertAlmostEqual(targets["right_hip"], -targets["left_hip"])
        self.assertAlmostEqual(targets["right_knee"], -targets["left_knee"])

    def test_crouching_reached_progressively(self):
        env = UpkieGyropod(self.servos_env, crouching_height=0.05)
        expected = self.servos_env.model.compute_crouching_posture(0.05)
        targets = self.leg_targets_after(env, nb_steps=1)
        for joint, target in targets.items():  # mock spine starts at zero
            self.assertGreater(abs(target), 0.0, msg=joint)
            self.assertLess(abs(target), abs(expected[joint]) / 10, msg=joint)

    def test_crouching_out_of_reach(self):
        with self.assertRaises(UpkieException):
            UpkieGyropod(self.servos_env, crouching_height=-0.01)
        with self.assertRaises(UpkieException):
            UpkieGyropod(
                self.servos_env, crouching_height=0.4
            )  # 2 ell = 0.343

    def test_crouching_outside_joint_limits(self):
        with self.assertRaises(UpkieException):  # hip 1.30 rad > 1.26 rad
            UpkieGyropod(self.servos_env, crouching_height=0.25)

    def test_set_crouching_height(self):
        env = UpkieGyropod(self.servos_env)
        env.set_crouching_height(0.05)
        self.assertEqual(env.crouching_height, 0.05)
        expected = self.servos_env.model.compute_crouching_posture(0.05)
        targets = self.leg_targets_after(env, nb_steps=1000)  # 10 s
        for joint, target in targets.items():
            self.assertAlmostEqual(target, expected[joint], places=4)

    def test_set_crouching_height_outside_joint_limits(self):
        env = UpkieGyropod(self.servos_env, crouching_height=0.05)
        with self.assertRaises(UpkieException):
            env.set_crouching_height(0.25)
        self.assertEqual(env.crouching_height, 0.05)  # unchanged

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
