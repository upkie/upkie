#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

import numpy as np


class WheeledInvertedPendulumReward:
    r"""!
    Weights of the position and velocity terms in rewards.
    """

    ## @var position_weight
    ## Weight of the position term.
    position_weight: float

    ## @var velocity_weight
    ## Weight of the velocity term.
    velocity_weight: float

    def __init__(
        self, position_weight: float = 1.0, velocity_weight: float = 1.0
    ):
        r"""!
        Initialize with reward weights.

        \param position Weight of the position term.
        \param velocity Weight of the velocity term.
        """
        self.position_weight = position_weight
        self.velocity_weight = velocity_weight

    def get(
        self,
        pitch: float,
        ground_position: float,
        angular_velocity: float,
        ground_velocity: float,
    ) -> float:
        r"""!
        Get reward for a given state.

        \param[in] pitch Pitch angle of the rotation from base to world, in
            [rad].
        \param[in] ground_position Position on the ground, in [m].
        \param[in] angular_velocity Angular velocity of the rotation from base
            to world, in [rad] / [s].
        \p[aram[in] ground_velocity Ground velocity, in [m] / [s].
        \return Reward.
        """
        tip_height = 0.58  # [m]
        tip_position = ground_position + tip_height * np.sin(pitch)
        tip_velocity = (
            ground_velocity + tip_height * angular_velocity * np.cos(pitch)
        )

        std_position = 0.05  # [m]
        position_reward = np.exp(-((tip_position / std_position) ** 2))
        velocity_penalty = -abs(tip_velocity)

        return (
            self.position_weight * position_reward
            + self.velocity_weight * velocity_penalty
        )
