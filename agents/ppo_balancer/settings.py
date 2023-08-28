#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from dataclasses import dataclass
from typing import Optional

import gin


@gin.configurable
@dataclass
class Settings:
    """Hyperparameters logged to TensorBoard."""

    agent_frequency: int
    effective_time_horizon: float
    max_episode_duration: float
    max_ground_accel: float
    max_ground_velocity: float
    spine_frequency: int
    total_timesteps: int


@gin.configurable
@dataclass
class PPOSettings:
    batch_size: int
    clip_range: float
    clip_range_vf: Optional[float]
    ent_coef: float
    gae_lambda: float
    learning_rate: float
    max_grad_norm: float
    n_epochs: int
    n_steps: int
    sde_sample_freq: int
    target_kl: Optional[float]
    use_sde: bool
    vf_coef: float
