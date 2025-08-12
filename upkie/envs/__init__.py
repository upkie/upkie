#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

## \namespace upkie.envs
## \brief Gymnasium environments.
##
## See the [list of environments](\ref gym-environments) for more details.

import gymnasium as gym

from .upkie_env import UpkieEnv
from .upkie_servos import UpkieServos


def register() -> None:
    """!
    Register Upkie environments with Gymnasium.

    Environments are named as `Upkie-<Backend>-<ActionSpace>`.
    """
    gym.envs.registration.register(
        id="Upkie-Mock-Pendulum",
        entry_point=("upkie.envs.entry_points:wrap_mock_pendulum"),
    )
    gym.envs.registration.register(
        id="Upkie-Mock-Servos",
        entry_point="upkie.envs.entry_points:make_mock_servos_env",
    )
    gym.envs.registration.register(
        id="Upkie-Spine-Pendulum",
        entry_point=("upkie.envs.entry_points:wrap_spine_pendulum"),
    )
    gym.envs.registration.register(
        id="Upkie-Spine-Servos",
        entry_point="upkie.envs.entry_points:make_spine_servos_env",
    )
    gym.envs.registration.register(
        id="Upkie-PyBullet-Pendulum",
        entry_point=("upkie.envs.entry_points:wrap_pybullet_pendulum"),
    )
    gym.envs.registration.register(
        id="Upkie-PyBullet-Servos",
        entry_point="upkie.envs.entry_points:make_pybullet_servos_env",
    )
    gym.envs.registration.register(
        id="Upkie-Genesis-Pendulum",
        entry_point=("upkie.envs.entry_points:wrap_genesis_pendulum"),
    )
    gym.envs.registration.register(
        id="Upkie-Genesis-Servos",
        entry_point="upkie.envs.entry_points:make_genesis_servos_env",
    )


__all__ = [
    "UpkieEnv",
    "UpkieServos",
    "register",
]
