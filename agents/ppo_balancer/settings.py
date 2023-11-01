#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria
# SPDX-License-Identifier: Apache-2.0

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import gin


@gin.configurable
@dataclass
class EnvSettings:
    """!
    Environment settings.
    """

    agent_frequency: int
    env_id: str
    max_ground_velocity: float
    spine_config: Dict
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
