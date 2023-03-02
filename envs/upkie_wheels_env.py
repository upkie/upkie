#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2022 Stéphane Caron
# Copyright 2023 Inria
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
import gymnasium as gym
import numpy as np
import yaml
from gymnasium import spaces
from vulp.spine import SpineInterface

from upkie_locomotion.observers.base_pitch import compute_base_pitch_from_imu

from .upkie_wheels_reward import UpkieWheelsReward

MAX_BASE_PITCH: float = np.pi
MAX_WHEEL_POSITION: float = float("inf")
MAX_IMU_ANGULAR_VELOCITY: float = 1000.0  # rad/s


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
        wheel_radius: Wheel radius in [m].
    """

    action_dict: dict
    action_dim: int
    config: dict
    fall_pitch: float
    max_ground_velocity: float
    observation_dict: dict
    observation_dim: int
    spine: SpineInterface
    wheel_radius: float

    def id(self) -> str:
        """
        Name and version of this environment for registration.

        Returns:
            Name and version of the environment.
        """
        return "UpkieWheelsEnv-v1"

    def __init__(
        self,
        config: Optional[dict],
        fall_pitch: float,
        max_ground_velocity: float,
        shm_name: str,
        wheel_radius: float,
    ):
        if config is None:
            envs_dir = path.dirname(__file__)
            with open(f"{envs_dir}/spine.yaml", "r") as fh:
                config = yaml.safe_load(fh)
        spine = SpineInterface(shm_name)

        action_dim = 1
        action_space = spaces.Box(
            -max_ground_velocity,
            +max_ground_velocity,
            shape=(action_dim,),
            dtype=np.float32,
        )

        observation_dim = 4
        max_wheel_velocity = max_ground_velocity / wheel_radius
        high = np.array(
            [
                MAX_BASE_PITCH,
                MAX_WHEEL_POSITION,
                max_wheel_velocity,
                MAX_IMU_ANGULAR_VELOCITY,
            ]
        )
        observation_space = spaces.Box(
            -high,
            +high,
            shape=(observation_dim,),
            dtype=np.float32,
        )

        # gym.Env members
        self.action_space = action_space
        self.observation_space = observation_space

        # Class members
        self.action_dict = {}
        self.action_dim = action_dim
        self.config = config
        self.fall_pitch = fall_pitch
        self.max_ground_velocity = max_ground_velocity
        self.observation_dim = observation_dim
        self.reward = UpkieWheelsReward()
        self.spine = spine
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

    def vectorize_observation(self, observation_dict: dict) -> np.ndarray:
        """
        Extract observation vector from a full observation dictionary.

        Args:
            observation_dict: Full observation dictionary from the spine.

        Returns:
            Observation vector.
        """
        imu = observation_dict["imu"]
        obs = np.empty(self.observation_dim)
        obs[0] = compute_base_pitch_from_imu(imu["orientation"])
        obs[1] = observation_dict["wheel_odometry"]["position"]
        obs[2] = observation_dict["wheel_odometry"]["velocity"]
        obs[3] = observation_dict["imu"]["angular_velocity"][1]
        return obs

    def reset(
        self,
        *,
        seed: Optional[int] = None,
        return_info: bool = False,
        options: Optional[dict] = None,
    ) -> Union[Any, Tuple[Any, Dict]]:
        """
        Resets the spine and get an initial observation.

        Returns:
            observation (object): the initial observation.
            info (optional dictionary): a dictionary containing extra
                information, this is only returned if return_info is set to
                true.
        """
        super().reset(seed=seed)
        self.spine.stop()
        self.spine.start(self.config)
        self.spine.get_observation()  # might be a pre-reset observation
        self.observation_dict = self.spine.get_observation()
        servo = self.observation_dict["servo"]
        self.action_dict = {
            "servo": {
                joint_name: {
                    "position": servo_observation["position"],
                    "velocity": 0.0,
                }
                for joint_name, servo_observation in servo.items()
            }
        }
        observation = self.observation_vector_from_dict(self.observation_dict)
        if not return_info:
            return observation
        else:  # return_info
            return observation, {}

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, dict]:
        """
        Run one timestep of the environment's dynamics. When end of episode is
        reached, you are responsible for calling `reset()` to reset the
        environment's state.

        Args:
            action: Action from the agent.

        Returns:
            observation: Agent's observation of the environment.
            reward: Amount of reward returned after previous action.
            done: Whether the episode has ended, in which case further step()
                calls will return undefined results.
            info: Contains auxiliary diagnostic information (helpful for
                debugging, logging, and sometimes learning).
        """
        commanded_velocity: float = action[0]
        left_wheel = self.action_dict["servo"]["left_wheel"]
        right_wheel = self.action_dict["servo"]["right_wheel"]
        left_wheel["position"] = math.nan
        right_wheel["position"] = math.nan
        left_wheel["velocity"] = +commanded_velocity / self.wheel_radius
        right_wheel["velocity"] = -commanded_velocity / self.wheel_radius
        self.spine.set_action(self.action_dict)

        # Observation
        self.observation_dict = self.spine.get_observation()
        observation = self.observation_vector_from_dict(self.observation_dict)

        # Reward
        reward = self.reward.get(observation)

        # Termination
        done = self.detect_fall(pitch=observation[0])
        return observation, reward, done, {}

    def detect_fall(self, pitch: float) -> bool:
        """
        Detect a fall based on the body-to-world pitch angle.

        Args:
            pitch: Current pitch angle in [rad].

        Returns:
            True if and only if a fall is detected.
        """
        return abs(pitch) > self.fall_pitch
