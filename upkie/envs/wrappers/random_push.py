#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

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
        if not env.has_wrapper_attr("set_bullet_action"):
            raise UpkieException(
                "Wrapped environment must have a `set_bullet_action` method"
            )
        self.env = env
        self.push_prob = push_prob
        self.push_generator = push_generator

    def step(self, action):
        """Adds a random push to the action."""
        if np.random.binomial(1, self.push_prob):
            force = self.push_generator()
            self.env.unwrapped.set_bullet_action(
                {"external_forces": {"torso": {"force": force}}}
            )
        else:  # Reset the forces to zero
            self.env.unwrapped.set_bullet_action(
                {"external_forces": {"torso": {"force": np.zeros(3)}}}
            )
        return self.env.step(action)
