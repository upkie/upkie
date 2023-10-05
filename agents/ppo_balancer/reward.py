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

    base_velocity_weight: float
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
        base_pitch = observation[0]
        # ground_position = observation[1]
        ground_velocity = observation[2]
        base_angular_velocity = observation[3]
        previous_commanded_velocity = observation[4]

        base_height = 0.3  # [m]
        base_velocity = (
            ground_velocity
            + base_angular_velocity * base_height * np.cos(base_pitch)
        )

        commanded_velocity = action[0]

        upright_reward = np.exp(-2.0 * base_pitch**2)
        base_velocity_penalty = -(base_velocity**2)
        ground_acceleration_penalty = -(
            (commanded_velocity - previous_commanded_velocity) ** 2
        )
        return (
            upright_reward
            + self.base_velocity_weight * base_velocity_penalty
            + self.ground_acceleration_weight * ground_acceleration_penalty
        )
