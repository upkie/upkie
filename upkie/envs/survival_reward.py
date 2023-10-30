#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria
# SPDX-License-Identifier: Apache-2.0

from typing import Tuple
from numpy.typing import NDArray

from .reward import Reward


class SurvivalReward(Reward):

    """!Reward function for staying alive as long as possible."""

    @staticmethod
    def get_range() -> Tuple[float, float]:
        """!
        Get range of the reward.

        This is part of the Gym API.
        """
        return (0.0, 2.0)

    def get(
        self, observation: NDArray[float], action: NDArray[float]
    ) -> float:
        """!
        Get reward corresponding to an observation.

        @param observation Observation to base the reward on.
        @param action Action to base the reward on.
        @returns Reward earned from executing the action from the observation.
        """
        return 1.0
