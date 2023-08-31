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

from typing import Tuple

import gin
import numpy as np

import upkie.envs


@gin.configurable
class Reward(upkie.envs.Reward):

    """!
    Reward function for balancing in place.

    This class has the following attributes:

    - ``lookahead_duration``: Length of the receding horizon, used to compute
      an internal divergent component of motion.
    - ``max_ground_accel``: Maximum ground acceleration in m/s^2.
    - ``max_pitch``: Maximum pitch angle we expect to observe, in [rad].
    - ``max_position``: Maximum ground position we expect to observe, in [m].
    - ``pitch_weight``: Weight of the pitch objective in the reward.
    - ``position_weight``: Weight of the position objective in the reward.
    """

    lookahead_duration: float
    max_ground_accel: float
    max_pitch: float
    max_position: float
    pitch_weight: float
    position_weight: float

    @staticmethod
    def get_range() -> Tuple[float, float]:
        """!
        Get range of the reward.

        This is part of the Gym API.
        """
        return (-float("inf"), 1.0)

    def __init__(
        self,
        action_weight: float,
        lookahead_duration: float,
        max_action: float,
        max_pitch: float,
        max_position: float,
        pitch_weight: float,
        position_weight: float,
    ):
        """!
        Initialize reward.

        @param lookahead_duration Length of the receding horizon, used to
            compute an internal divergent component of motion.
        @param max_pitch Maximum pitch angle for standardization, in [rad].
        @param max_position Large ground position for standardization, in [m].
        @param pitch_weight Weight of the pitch objective in the reward.
        @param position_weight Weight of the position objective in the reward.
        """
        self.action_weight = action_weight
        self.lookahead_duration = lookahead_duration
        self.max_action = max_action
        self.max_pitch = max_pitch
        self.max_position = max_position
        self.pitch_weight = pitch_weight
        self.position_weight = position_weight

    def get(self, observation: np.ndarray, action: np.ndarray) -> float:
        """!
        Get reward corresponding to an observation.

        @param observation Observation to base the reward on.
        @param action Action to base the reward on.
        @returns Reward earned from executing the action from the observation.
        """
        pitch = observation[0]
        ground_position = observation[1]
        ground_velocity = observation[2]
        angular_velocity = observation[3]

        T = self.lookahead_duration
        lookahead_pitch = pitch + T * angular_velocity
        lookahead_position = ground_position + T * ground_velocity
        normalized_lookahead_pitch = lookahead_pitch / self.max_pitch
        normalized_lookahead_position = lookahead_position / self.max_position
        normalized_action = action[0] / self.max_action
        return (
            1.0
            - self.pitch_weight * abs(normalized_lookahead_pitch)
            - self.position_weight * abs(normalized_lookahead_position)
            - self.action_weight * normalized_action
        )
