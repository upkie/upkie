#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

from typing import Any, Dict, SupportsFloat, Tuple

import gymnasium as gym
from gymnasium.core import ActType, ObsType


class ObservationBasedReward(gym.Wrapper[ObsType, ActType, ObsType, ActType]):
    r"""!
    Redefine the reward of an environment as a function of its observation.

    If you would like to redefine the reward that is returned by the base
    environment before passing it to learning code, you can simply inherit from
    this class and overwrite the :meth:`reward` method.

    See also the (reward-based) `RewardWrapper` in Gymnasium.
    """

    def __init__(self, env: gym.Env[ObsType, ActType]):
        r"""!
        Constructor for the Reward wrapper.

        \param[in] env Environment to be wrapped.
        """
        super().__init__(self, env)

    def step(
        self, action: ActType
    ) -> Tuple[ObsType, SupportsFloat, bool, bool, Dict[str, Any]]:
        r"""!
        Modifies the :attr:`env` reward using :meth:`self.reward`.
        """
        observation, _, terminated, truncated, info = self.env.step(action)
        return (
            observation,
            self.reward(observation),
            terminated,
            truncated,
            info,
        )

    def reward(self, observation: ObsType) -> SupportsFloat:
        r"""!Returns the new environment reward.

        \param[in] observation Latest observation from the environment.
        \return The modified reward.
        """
        raise NotImplementedError
