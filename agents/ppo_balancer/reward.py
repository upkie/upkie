#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
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

import gin
import numpy as np

import upkie.envs


@gin.configurable
class Reward(upkie.envs.Reward):

    """!
    Reward function for balancing in place.
    """

    def __init__(self, acceleration_penalty_weight: float):
        """!
        Initialize reward.
        """
        self.acceleration_penalty_weight = acceleration_penalty_weight

    def get(self, observation: np.ndarray, action: np.ndarray) -> float:
        """!
        Get reward corresponding to an observation.

        @param observation Observation to base the reward on.
        @param action Action to base the reward on.
        @returns Reward earned from executing the action from the observation.
        """
        # pitch = observation[0]
        # ground_position = observation[1]
        ground_velocity = observation[2]
        # abs_angular_velocity = abs(observation[3])

        commanded_velocity = action[0]

        # low_velocity = np.exp(-1.5 * ground_velocity**2)
        # low_velocity = -(ground_velocity**2)
        low_acceleration = np.exp(
            -1.5 * (commanded_velocity - ground_velocity) ** 2
        )
        return 1.0 + self.acceleration_penalty_weight * low_acceleration
