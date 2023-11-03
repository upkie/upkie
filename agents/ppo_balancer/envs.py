#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria
# SPDX-License-Identifier: Apache-2.0

from typing import List, Tuple

import gin
import gymnasium
import numpy as np
from gymnasium import spaces
from gymnasium.wrappers import RescaleAction

from upkie.envs import UpkieGroundVelocity
from upkie.envs.wrappers import (
    DifferentiateAction,
    LowPassFilterAction,
    NoisifyAction,
    NoisifyObservation,
)


@gin.configurable
def make_training_env(
    velocity_env: UpkieGroundVelocity,
    observation_noise: List[float],
    action_lpf: Tuple[float, float],
    action_noise: List[float],
) -> gymnasium.Wrapper:
    action_noise = np.array(action_noise)
    observation_noise = np.array(observation_noise)
    noisy_obs_env = NoisifyObservation(velocity_env, noise=observation_noise)
    noisy_env = NoisifyAction(noisy_obs_env, noise=action_noise)
    filtered_env = LowPassFilterAction(
        noisy_env,
        time_constant=spaces.Box(*action_lpf),
    )
    return filtered_env


@gin.configurable
def make_accel_env(
    velocity_env: UpkieGroundVelocity,
    training: bool,
    max_ground_accel: float,
    accel_penalty: float,
) -> gymnasium.Wrapper:
    inner_env = make_training_env(velocity_env) if training else velocity_env
    accel_env = DifferentiateAction(
        inner_env,
        min_derivative=-max_ground_accel,
        max_derivative=+max_ground_accel,
    )
    rescaled_accel_env = RescaleAction(
        accel_env,
        min_action=-1.0,
        max_action=+1.0,
    )
    return rescaled_accel_env
