#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

from typing import Tuple, Union

import gymnasium
import numpy as np
from gymnasium.spaces import Box

from upkie.utils.filters import low_pass_filter


class AddLagToAction(gymnasium.Wrapper):
    """!
    Model lag by applying a low-pass filter to the action of an environment.

    Note that there is a difference between "delay" and "lag":

    - Delay is a fixed time interval corresponding to the time it takes the
      input to affect the output, e.g. `input[t] = output[t + delay]`.
    - Lag is a phase shift in the system's response. It can be thought of as a
      gradual response to input changes where the output does not immediately
      match the input.

    In this wrapper, we model lag where the output (action forward to the
    wrapped environment) is a low-pass filtered version of the input (action
    passed to `step`).
    """

    ## \var filtered_action
    ## Wrapped action after low-pass filtering.
    filtered_action: np.ndarray

    ## \var time_constant
    ## Cutoff period in seconds of a low-pass filter applied to the action.
    time_constant: float

    ## \var time_constant_box
    ## Box space from which the time constant is sampled at every reset of the
    ## environment.
    time_constant_box: Box

    def __init__(self, env, time_constant: Union[float, Box]):
        r"""!
        Initialize wrapper.

        \param env Environment to wrap.
        \param time_constant Cutoff period in seconds of the low-pass filter
            applied to the action. If a Box is provided, couple of lower and
            upper bounds for the action: a new time constant is sampled
            uniformly at random between these bounds at every reset of the
            environment.
        """
        super().__init__(env)
        time_constant_box = (
            time_constant
            if isinstance(time_constant, Box)
            else Box(low=time_constant - 1e-10, high=time_constant + 1e-10)
        )
        self.filtered_action = np.zeros(env.action_space.shape)
        self.time_constant = time_constant_box.sample()
        self.time_constant_box = time_constant_box

    def reset(self, **kwargs):
        r"""!
        Reset the environment.

        \param kwargs Keyword arguments forwarded to the wrapped environment.
        """
        self.filtered_action = np.zeros(self.env.action_space.shape)
        self.time_constant = self.time_constant_box.sample()
        return self.env.reset(**kwargs)

    def step(
        self, action: np.ndarray
    ) -> Tuple[np.ndarray, float, bool, bool, dict]:
        r"""!
        Step the environment.

        \param action Action from the agent.
        \return Tuple with (observation, reward, terminated, truncated,info).
            See \ref upkie.envs.upkie_base_env.UpkieBaseEnv.step for details.
        """
        dt = self.env.unwrapped.dt
        if self.time_constant <= 2.0 * dt:
            # Nyquist–Shannon sampling theorem
            return self.env.step(action)

        self.filtered_action = low_pass_filter(
            self.filtered_action,
            self.time_constant,
            action,
            dt,
        )
        return self.env.step(self.filtered_action)
