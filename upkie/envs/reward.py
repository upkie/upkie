#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria
# SPDX-License-Identifier: Apache-2.0

import abc

import numpy as np


class Reward:

    """!
    Reward function \f$R(s, a)\f$.
    """

    @abc.abstractmethod
    def get(self, observation: np.ndarray, action: np.ndarray) -> float:
        """!
        Get reward corresponding to an observation.

        @param observation Observation to base the reward on.
        @param action Action to base the reward on.
        @returns Reward earned from executing the action from the observation.
        """
