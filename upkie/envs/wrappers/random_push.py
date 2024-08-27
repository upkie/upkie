#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

import numpy as np
from gymnasium import Env, Wrapper


class RandomPush(Wrapper):
    """!
    At each step, with a given probability, apply a random push to the robot.
    """

    ## \var env
    ## Wrapped environment.
    env: Env

    # Internal reference to unwrapped get_spine_action function.
    _env_get_spine_action: callable

    ## \var push_generator
    ## Function that generates the push force. It should return a 3D numpy
    ## array.
    push_generator: callable

    ## \var push_prob
    ## Probability of pushing at each step.
    push_prob: float

    def __init__(
        self,
        env,
        push_prob=0.01,
        push_generator=lambda: np.random.normal(0, 400, 3),
    ):
        r"""!
        Initialize wrapper.

        \param env Environment to wrap.
        \param push_prob Probability of pushing at each step.
        \param push_generator Function that generates the push force. It should
            return a 3D numpy array.
        """
        super().__init__(env)
        self.env = env
        self.push_prob = push_prob
        self.push_generator = push_generator
        if not hasattr(self.env, "get_spine_action"):
            raise ValueError(
                "The environment must have a method get_spine_action"
            )
        self._env_get_spine_action = self.env.get_spine_action
        self.env.unwrapped.get_spine_action = self.get_spine_action

    def get_spine_action(self, action):
        """Adds a random push to the action."""
        spine_action = self._env_get_spine_action(action)
        if np.random.binomial(1, self.push_prob):
            force = self.push_generator()
            spine_action["bullet"] = {
                "external_forces": {"torso": {"force": force}}
            }
        else:  # Reset the forces to zero
            spine_action["bullet"] = {
                "external_forces": {"torso": {"force": np.zeros(3)}}
            }
        return spine_action
