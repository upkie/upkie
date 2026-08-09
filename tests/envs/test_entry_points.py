# SPDX-License-Identifier: Apache-2.0

"""Test Gymnasium environment entry points."""

import unittest
from multiprocessing.shared_memory import SharedMemory

import gymnasium as gym
import numpy as np

from upkie.exceptions import UpkieTimeoutError


class EntryPointsTestCase(unittest.TestCase):
    def test_mock_servos(self):
        with gym.make("Upkie-Mock-Servos") as env:
            self.assertIsNotNone(env)

    def test_mock_pendulum(self):
        with gym.make("Upkie-Mock-Pendulum") as env:
            self.assertIsNotNone(env)

    def test_spine_servos(self):
        shm = SharedMemory(name=None, size=42, create=True)
        try:
            env = gym.make("Upkie-Spine-Servos", shm_name=shm._name)
            self.assertIsNotNone(env)
            try:
                del env  # we delete it explicitly
            except UpkieTimeoutError:  # to catch this exception
                pass  # which is ok: there is no spine, thus no response
        finally:
            shm.close()

    def test_spine_pendulum(self):
        shm = SharedMemory(name=None, size=42, create=True)
        try:
            env = gym.make("Upkie-Spine-Pendulum", shm_name=shm._name)
            self.assertIsNotNone(env)
            try:
                del env  # we delete it explicitly
            except UpkieTimeoutError:  # to catch this exception
                pass  # which is ok: there is no spine, thus no response
        finally:
            shm.close()

    def test_pybullet_servos(self):
        with gym.make("Upkie-PyBullet-Servos", gui=False) as env:
            self.assertIsNotNone(env)

    def test_pybullet_pendulum(self):
        with gym.make("Upkie-PyBullet-Pendulum", gui=False) as env:
            self.assertIsNotNone(env)

    def test_mock_gyropod(self):
        with gym.make("Upkie-Mock-Gyropod") as env:
            self.assertIsNotNone(env)

    def test_spine_gyropod(self):
        shm = SharedMemory(name=None, size=42, create=True)
        try:
            env = gym.make("Upkie-Spine-Gyropod", shm_name=shm._name)
            self.assertIsNotNone(env)
            try:
                del env
            except UpkieTimeoutError:
                pass
        finally:
            shm.close()

    def test_pybullet_gyropod(self):
        with gym.make("Upkie-PyBullet-Gyropod", gui=False) as env:
            self.assertIsNotNone(env)

    def test_cookie_mock_servos_uses_cookie_model(self):
        with gym.make("Cookie-Mock-Servos") as env:
            self.assertIn("cookie", env.unwrapped.model.urdf_path)

    @unittest.skip(
        "cookie_description's imu_placement joint currently yields the "
        "same rotation_base_to_imu as the stock Upkie mounting "
        "(diag(-1, 1, -1)), which is wrong for Cookie's actual IMU "
        "mounting (should be [-1, 0, 0, 0, 0, 1, 0, 1, 0], see "
        "~/.config/upkie/config.yml's base_orientation override on the "
        "real robot). Re-enable once cookie_description's URDF is fixed "
        "upstream."
    )
    def test_cookie_and_upkie_base_orientations_differ(self):
        """Cookie's IMU is not mounted the same way as the stock Upkie."""
        with gym.make("Upkie-Mock-Servos") as upkie_env:
            upkie_rotation = upkie_env.unwrapped.model.rotation_base_to_imu
        with gym.make("Cookie-Mock-Servos") as cookie_env:
            cookie_rotation = cookie_env.unwrapped.model.rotation_base_to_imu
        self.assertFalse(np.array_equal(upkie_rotation, cookie_rotation))

    def test_unregistered(self):
        with self.assertRaises(gym.error.NameNotFound):
            gym.make("Upkie-Servos-NotFound")
        with gym.make("Upkie-Mock-Servos") as env:
            self.assertIsNotNone(env)


if __name__ == "__main__":
    unittest.main()
