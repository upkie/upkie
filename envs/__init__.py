#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2022 Stéphane Caron
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

import gym

from .upkie_base_env import UpkieBaseEnv
from .upkie_servos_env import UpkieServosEnv
from .upkie_wheels_env import UpkieWheelsEnv


def register():
    gym.envs.register(
        id=f"UpkieServosEnv-v{UpkieServosEnv.version}",
        entry_point="upkie_locomotion.envs:UpkieServosEnv",
        max_episode_steps=1_000_000_000,
    )
    gym.envs.register(
        id=f"UpkieWheelsEnv-v{UpkieWheelsEnv.version}",
        entry_point="upkie_locomotion.envs:UpkieWheelsEnv",
        max_episode_steps=1_000_000_000,
    )


__all__ = [
    "UpkieBaseEnv",
    "UpkieWheelsEnv",
    "register",
]
