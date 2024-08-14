#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

import gymnasium
import numpy as np
from gymnasium import spaces

from upkie.exceptions import UpkieException


class AddActionToObservation(gymnasium.ObservationWrapper):
    """!
    Append last action vector to the observation vector.

    Can be used in combination with gymnasium.wrappers.FrameStackObservation to
    create histories. (If there were directly a history wrapper in gymnasium we
    wouldn't need this wrapper.)
    """

    _last_action: np.ndarray

    def __init__(self, env):
        r"""!
        Initialize wrapper.

        \param env Wrapped environment.

        The initial action is set to zero.
        """
        super().__init__(env)
        if env.observation_space.dtype != env.action_space.dtype:
            raise UpkieException(
                "Not sure which type to pick "
                f"between {env.observation_space.dtype=} "
                f"and {env.action_space.dtype=}"
            )
        low = np.concatenate([env.observation_space.low, env.action_space.low])
        self.observation_space = spaces.Box(
            low=low,
            high=np.concatenate(
                [env.observation_space.high, env.action_space.high]
            ),
            shape=low.shape,
            dtype=env.observation_space.dtype,
        )
        self._last_action = np.zeros(
            env.action_space.shape, dtype=env.action_space.dtype
        )

    def observation(self, observation: np.ndarray) -> np.ndarray:
        return np.concatenate([observation, self._last_action])

    def step(self, action: np.ndarray):
        self._last_action = action
        return super().step(action)
