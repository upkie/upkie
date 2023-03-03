#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria

from dataclasses import dataclass
from typing import Optional

import gin


@gin.configurable
@dataclass
class Settings:
    """Hyperparameters logged to TensorBoard."""

    batch_size: int
    brain_frequency: int
    clip_range: float
    clip_range_vf: Optional[float]
    effective_time_horizon: float
    ent_coef: float
    gae_lambda: float
    learning_rate: float
    max_grad_norm: float
    n_epochs: int
    n_steps: int
    sde_sample_freq: int
    spine_frequency: int
    target_kl: Optional[float]
    total_timesteps: int
    use_sde: bool
    vf_coef: float
