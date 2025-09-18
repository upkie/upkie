#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

## \namespace upkie.envs.wrappers.random_push
## \brief Apply a random push at each step with a given probability.

import gymnasium as gym
import numpy as np

from upkie.exceptions import UpkieException


class RandomPush(gym.Wrapper):
    """!
    At each step, with a given probability, apply a random push to the robot.
    """

    ## \var env
    ## Wrapped environment.
    env: gym.Env

    ## \var push_generator
    ## Function that generates the push force. It should return a 3D numpy
    ## array.
    push_generator: callable

    ## \var push_prob
    ## Probability of pushing at each step.
    push_prob: float

    def __init__(
        self,
        env: gym.Env,
        push_prob: float = 0.01,
        push_generator: callable = lambda: np.random.normal(0, 400, 3),
    ):
        r"""!
        Initialize wrapper.

        \param env Environment to wrap.
        \param push_prob Probability of applying a push at each timestep.
        \param push_generator Function that generates the push force. It should
            return a 3D NumPy array.
        """
        super().__init__(env)
        if not hasattr(env.backend, "set_external_forces"):
            raise UpkieException(
                "Wrapped environment backend must have a "
                "`set_external_forces` method"
            )
        self.env = env
        self.push_prob = push_prob
        self.push_generator = push_generator

    def step(self, action):
        r"""!
        Adds a random push to the action.
        """
        if np.random.binomial(1, self.push_prob):
            force = self.push_generator()
            external_forces = {"torso": {"force": force, "local": False}}
        else:  # Reset the forces to zero
            external_forces = {"torso": {"force": np.zeros(3), "local": False}}

        self.env.backend.set_external_forces(external_forces)
        return self.env.step(action)
