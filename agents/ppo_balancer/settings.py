#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria

from dataclasses import dataclass
from typing import Optional


@dataclass
class Settings:
    """Hyperparameters logged to TensorBoard."""

    agent_frequency: int = 200
    batch_size: int = 64
    clip_range: float = 0.2
    clip_range_vf: Optional[float] = None
    effective_time_horizon: float = 0.5  # [s]
    ent_coef: float = 0.0
    gae_lambda: float = 0.95
    learning_rate: float = 3e-4
    max_episode_duration: float = 10.0  # [s]
    max_grad_norm: float = 0.5
    n_epochs: int = 10
    n_steps: int = 2048
    sde_sample_freq: int = -1
    spine_frequency: int = 1000
    target_kl: Optional[float] = None
    total_timesteps: int = 5e5
    use_sde: bool = False
    vf_coef : float = 0.5
