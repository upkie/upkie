#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2022 Stéphane Caron
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
from os import path
from typing import Any, Dict, Optional, Tuple, Union

import gin
import gym
import numpy as np
import yaml
from gym import spaces
from vulp.spine import SpineInterface

from upkie_locomotion.observers.base_pitch import compute_base_pitch_from_imu

from .reward import Reward


@gin.configurable
class UpkieWheelsEnv(gym.Env):

    """
    Upkie with full observation but only wheel velocity actions.

    Attributes:
        action_dict: Dictionary of actions to send to the spine.
        action_dim: Dimension of action space.
        config: Configuration dictionary, also sent to the spine.
        fall_pitch: Fall pitch angle, in radians.
        max_ground_velocity: Maximum commanded ground velocity in [m] / [s].
        observation_dict: Dictionary of last observation from the spine.
        observation_dim: Dimension of observation space.

            =====  ============================================================
            Index  Description
            =====  ============================================================
            0      | Base pitch in rad.
            1      | Position of the average wheel contact point, in m.
            2      | Velocity of the average wheel contact point, in m/s.
            3      | Body angular velocity of the IMU frame along its y-axis,
                   | in rad/s.
            =====  ============================================================

        spine: Interface to the spine.
        version: Version of the environment for registration.
        wheel_radius: Wheel radius in [m].
    """

    action_dict: dict
    config: dict
    fall_pitch: float
    max_ground_velocity: float
    observation_dict: dict
    spine: SpineInterface
    version: int
    wheel_radius: float

    def id(self) -> str:
        """
        Name and version of this environment for registration.

        Returns:
            Name and version of the environment.
        """
        return f"UpkieWheelsEnv-v{self.version}"

    def __init__(
        self,
        reward: Optional[Reward],
        config: Optional[dict],
        fall_pitch: float,
        max_ground_velocity: float,
        shm_name: str,
        version: int,
        wheel_radius: float,
    ):
        if config is None:
            agent_dir = path.dirname(__file__)
            with open(f"{agent_dir}/spine.yaml", "r") as fh:
                config = yaml.safe_load(fh)
        spine = SpineInterface(shm_name)

        action_dim = 1
        action_space = spaces.Box(
            -max_ground_velocity,
            +max_ground_velocity,
            shape=(action_dim,),
            dtype=np.float32,
        )

        high = []
        observation_dim = 0
        if version >= 1:
            observation_dim += 1
            high.append(np.pi)
        if version >= 2:
            observation_dim += 1
            high.append(float("inf"))
        if version >= 3:
            observation_dim += 1
            max_wheel_velocity = max_ground_velocity / wheel_radius
            high.append(max_wheel_velocity)
        if version >= 4:
            observation_dim += 1
            MAX_IMU_ANGULAR_VELOCITY = 1000.0  # rad/s
            high.append(MAX_IMU_ANGULAR_VELOCITY)
        high = np.array(high)
        observation_space = spaces.Box(
            -high,
            +high,
            shape=(observation_dim,),
            dtype=np.float32,
        )

        # gym.Env members
        self.action_space = action_space
        self.observation_space = observation_space
        self.reward_range = reward.get_range()

        # Class members
        self.action_dict = {}
        self.action_dim = action_dim
        self.config = config
        self.fall_pitch = fall_pitch
        self.max_ground_velocity = max_ground_velocity
        self.observation_dict = {}
        self.observation_dim = observation_dim
        self.observation_dim = observation_dim
        self.reward = reward
        self.spine = spine
        self.version = version
        self.wheel_radius = wheel_radius

    def close(self) -> None:
        """
        Stop the spine properly.
        """
        self.spine.stop()

    @staticmethod
    def gin_config():
        """
        Path to the Gin configuration for this environment.
        """
        dirname = path.dirname(__file__)
        basename = path.basename(__file__).replace(".py", ".gin")
        return f"{dirname}/{basename}"

    def detect_fall(self, pitch: float) -> bool:
        """
        Detect a fall based on the body-to-world pitch angle.

        Args:
            pitch: Current pitch angle in [rad].

        Returns:
            True if and only if a fall is detected.
        """
        return abs(pitch) > self.fall_pitch
