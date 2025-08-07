#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

import gymnasium as gym
import numpy as np
from gymnasium import spaces


class ActionObserverEnv(gym.Env):
    r"""!
    Test environment that returns the action as observation.

    This environment is useful for testing action wrappers, as it allows
    verification that the wrapper correctly modifies actions by observing
    the output.
    """

    def __init__(self):
        action_space = spaces.Box(0.0, 2.0, shape=(1,))
        self.action_space = action_space
        self.observation_space = action_space
        self.dt = 1e-3

    def step(self, action):
        r"""!
        Step the environment by returning the action as observation.

        \param action The action to take.

        \return A tuple of (observation, reward, terminated, truncated, info)
            where observation equals the input action.
        """
        observation = action
        return observation, 0.0, False, False, {}


class ConstantObservationEnv(gym.Env):
    r"""!
    Test environment that returns a constant observation.

    This environment always returns the same observation value regardless
    of the action taken. It also stores PyBullet actions for testing purposes.
    """

    def __init__(self, constant: float):
        self.action_space = spaces.Box(-1.0, 1.0, shape=(1,))
        self.observation_space = spaces.Box(0.0, 2.0, shape=(1,))
        self.constant = constant
        self.__bullet_action = {}

    def step(self, action):
        r"""!
        Step the environment by returning a constant observation.

        \param action The action to take (ignored).

        \return A tuple of (observation, reward, terminated, truncated, info)
            where observation is always the constant value.
        """
        observation = np.array([self.constant])
        return observation, 0.0, False, False, {}

    def get_bullet_action(self) -> dict:
        r"""!
        Get the stored PyBullet action.

        \return The stored PyBullet action dictionary.
        """
        return self.__bullet_action

    def set_bullet_action(self, bullet_action: dict) -> None:
        r"""!
        Set the PyBullet action to be stored.

        \param bullet_action The PyBullet action dictionary to store.
        """
        self.__bullet_action = bullet_action.copy()


class MockSpine:
    r"""!
    Mock spine interface for testing Upkie environments.

    This class simulates the behavior of a real spine interface without
    requiring actual hardware or external processes. It provides mock
    observations and handles actions for testing purposes.
    """

    def __init__(self):
        self.observation = {
            "base_orientation": {
                "pitch": 0.1,
                "angular_velocity": [-2e-3, 3e2, 1e-8],
                "linear_velocity": [1e3, 2e2, 3e1],
            },
            "imu": {
                "orientation": [1.0, 0.0, 0.0, 0.0],
                "angular_velocity": [0.0, 0.0, 0.0],
                "linear_acceleration": [0.0, 0.0, 0.0],
            },
            "number": 0,
            "servo": {
                f"{side}_{joint}": {
                    "position": 0.0,
                    "velocity": 0.0,
                    "torque": 0.0,
                    "temperature": 42.0,
                    "voltage": 18.0,
                }
                for side in ("left", "right")
                for joint in ("hip", "knee", "wheel")
            },
            "wheel_odometry": {
                "position": 0.0,
                "velocity": 0.0,
            },
        }

    def _next_observation(self) -> dict:
        self.observation["number"] += 1
        return self.observation

    def start(self, config: dict) -> dict:
        r"""!
        Start the mock spine.

        \param config Configuration dictionary (ignored).

        \return The initial observation dictionary.
        """
        return self._next_observation()

    def stop(self) -> None:
        r"""!
        Stop the mock spine.

        This is a no-op for the mock spine.
        """
        pass

    def set_action(self, action) -> dict:
        r"""!
        Set an action and get the next observation.

        \param action The action to execute.

        \return The next observation dictionary.
        """
        self.action = action
        return self._next_observation()
