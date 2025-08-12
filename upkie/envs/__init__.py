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
        entry_point="upkie.envs.upkie_mock_env:UpkieMockEnv",
    )
    gym.envs.registration.register(
        id="Upkie-Spine-Pendulum",
        entry_point=("upkie.envs.entry_points:wrap_spine_pendulum"),
    )
    gym.envs.registration.register(
        id="Upkie-Spine-Servos",
        entry_point="upkie.envs.upkie_spine_env:UpkieSpineEnv",
    )
    gym.envs.registration.register(
        id="Upkie-PyBullet-Pendulum",
        entry_point=("upkie.envs.entry_points:wrap_pybullet_pendulum"),
    )
    gym.envs.registration.register(
        id="Upkie-PyBullet-Servos",
        entry_point="upkie.envs.upkie_pybullet_env:UpkiePyBulletEnv",
    )
    gym.envs.registration.register(
        id="Upkie-Genesis-Pendulum",
        entry_point=("upkie.envs.entry_points:wrap_genesis_pendulum"),
    )
    gym.envs.registration.register(
        id="Upkie-Genesis-Servos",
        entry_point="upkie.envs.upkie_genesis_env:UpkieGenesisEnv",
    )


__all__ = [
    "register",
]
