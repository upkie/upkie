#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

from dataclasses import dataclass
from typing import List, Optional, Tuple

import gin


@gin.configurable
@dataclass
class EnvSettings:
    """!
    Environment settings.
    """

    accel_penalty: float
    action_lpf: Tuple[float, float]
    action_noise: List[float]
    agent_frequency: int
    env_id: str
    history_size: int
    max_ground_accel: float
    max_ground_velocity: float
    observation_noise: List[float]
    reward_weights: dict
    spine_config: dict
    spine_frequency: int


@gin.configurable
@dataclass
class PPOSettings:
    """PPO algorithm settings."""

    batch_size: int
    clip_range: float
    clip_range_vf: Optional[float]
    ent_coef: float
    gae_lambda: float
    learning_rate: float
    max_grad_norm: float
    n_epochs: int
    n_steps: int
    net_arch_pi: Tuple[int, int]
    net_arch_vf: Tuple[int, int]
    normalize_advantage: bool
    sde_sample_freq: int
    target_kl: Optional[float]
    use_sde: bool
    vf_coef: float


@gin.configurable
@dataclass
class TrainingSettings:
    """Training settings."""

    init_rand: dict
    max_episode_duration: float
    return_horizon: float
    total_timesteps: int
