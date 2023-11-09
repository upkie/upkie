#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria
# SPDX-License-Identifier: Apache-2.0

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

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
    reward_weights: Dict[str, Any]
    spine_config: Dict[str, Any]
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
class SACSettings:
    """SAC algorithm settings."""

    action_noise: Optional[float]
    batch_size: int
    buffer_size: int
    ent_coef: str
    gradient_steps: int
    learning_rate: float
    learning_starts: int
    net_arch_pi: Tuple[int, int]
    net_arch_qf: Tuple[int, int]
    optimize_memory_usage: bool
    sde_sample_freq: int
    stats_window_size: int
    target_entropy: str
    target_update_interval: int
    tau: float
    train_freq: int
    use_sde: bool
    use_sde_at_warmup: bool


@gin.configurable
@dataclass
class TrainingSettings:
    """Training settings."""

    init_rand: Dict[str, float]
    max_episode_duration: float
    return_horizon: float
    total_timesteps: float
