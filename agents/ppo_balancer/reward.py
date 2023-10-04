#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria
# SPDX-License-Identifier: Apache-2.0

from dataclasses import dataclass

import gin
import numpy as np
from numpy.typing import NDArray

import upkie.envs


@gin.configurable
@dataclass
class Reward(upkie.envs.Reward):

    """!
    Reward function for balancing in place.
    """

    ground_velocity_weight: float
    ground_acceleration_weight: float

    def get(
        self, observation: NDArray[float], action: NDArray[float],
    ) -> float:
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
        previous_commanded_velocity = observation[4]

        commanded_velocity = action[0]

        ground_velocity_reward = np.exp(-1.5 * ground_velocity**2)
        ground_acceleration_reward = np.exp(
            -1.5 * (commanded_velocity - previous_commanded_velocity) ** 2
        )
        return (
            1.0
            + self.ground_velocity_weight * ground_velocity_reward
            + self.ground_acceleration_weight * ground_acceleration_reward
        )
