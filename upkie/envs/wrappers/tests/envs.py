#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

import gymnasium as gym
import numpy as np
from gymnasium import spaces


class ActionObserverEnv(gym.Env):
    def __init__(self):
        action_space = spaces.Box(0.0, 2.0, shape=(1,))
        self.action_space = action_space
        self.observation_space = action_space
        self.dt = 1e-3

    def step(self, action):
        observation = action
        return observation, 0.0, False, False, {}


class ConstantObservationEnv(gym.Env):
    def __init__(self, constant: float):
        self.action_space = spaces.Box(-1.0, 1.0, shape=(1,))
        self.observation_space = spaces.Box(0.0, 2.0, shape=(1,))
        self.constant = constant
        self.__bullet_action = {}

    def step(self, action):
        observation = np.array([self.constant])
        return observation, 0.0, False, False, {}

    def get_bullet_action(self) -> dict:
        return self.__bullet_action

    def set_bullet_action(self, bullet_action: dict) -> None:
        self.__bullet_action = bullet_action.copy()
