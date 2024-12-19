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
    this class and overwrite the :meth:`reward` method. See also the
    `RewardWrapper` class defined in Gymnasium.

    Rewards in reinforcement learning are often defined as \f$r(s_t, a_t,
    s_{t+1})\f$. With this wrapper, the reward function is redefined as
    \f$r(o_{t+1})\f$, with two differences:

    - We use the observation rather than the environment state.
    - The new reward is computed based on the post-step observation.

    Using observation rather than state does not have a big impact when
    training in simulation, as simulation sensors can read any part of the full
    simulation state on demand.
    """

    def __init__(self, env: gym.Env[ObsType, ActType]):
        r"""!
        Constructor for the Reward wrapper.

        \param[in] env Environment to be wrapped.
        """
        super().__init__(env)

    def step(
        self, action: ActType
    ) -> Tuple[ObsType, SupportsFloat, bool, bool, Dict[str, Any]]:
        r"""!
        Modifies the :attr:`env` reward using :meth:`self.reward`.
        """
        observation, _, terminated, truncated, info = self.env.step(action)
        return (
            observation,
            self.reward(observation, info),
            terminated,
            truncated,
            info,
        )

    def reward(self, observation: ObsType, info: dict) -> SupportsFloat:
        r"""!
        Returns the new environment reward.

        \param[in] observation Latest observation from the environment.
        \param[in] info Latest info dictionary from the environment.
        \return The modified reward.
        """
        raise NotImplementedError
