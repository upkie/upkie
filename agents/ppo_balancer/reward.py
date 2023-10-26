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

    ground_acceleration_weight: float
    pitch_weight: float
    torso_height: float
    torso_position_weight: float
    torso_velocity_weight: float

    def get(
        self, observation: NDArray[float], action: NDArray[float],
    ) -> float:
        """!
        Get reward corresponding to an observation.

        @param observation Observation to base the reward on.
        @param action Action to base the reward on.
        @returns Reward earned from executing the action from the observation.
        """
        torso_height = self.torso_height
        torso_pitch = observation[0]
        ground_position = observation[1]
        ground_velocity = observation[2]
        torso_angular_velocity = observation[3]
        # last_commanded_velocity = observation[4]
        commanded_acceleration = action[0]

        torso_position = ground_position + torso_height * np.sin(torso_pitch)
        torso_velocity = (
            ground_velocity
            + torso_height * torso_angular_velocity * np.cos(torso_pitch)
        )

        ground_acceleration_penalty = -(commanded_acceleration**2)
        pitch_reward = np.exp(-2.0 * torso_pitch**2)
        torso_position_reward = np.exp(-2.0 * torso_position**2)
        torso_velocity_penalty = -(torso_velocity**2)

        return (
            0.0
            + self.ground_acceleration_weight * ground_acceleration_penalty
            + self.pitch_weight * pitch_reward
            + self.torso_position_weight * torso_position_reward
            + self.torso_velocity_weight * torso_velocity_penalty
        )
