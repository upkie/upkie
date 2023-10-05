#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria
# SPDX-License-Identifier: Apache-2.0

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
