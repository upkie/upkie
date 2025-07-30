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

    Environments are named as `Upkie-<Interface>-<ActionSpace>`.
    """
    gym.envs.registration.register(
        id="Upkie-Mock-GroundVelocity",
        entry_point=("upkie.envs.entry_points:make_mock_ground_velocity"),
    )
    gym.envs.registration.register(
        id="Upkie-Mock-Servos",
        entry_point="upkie.envs.upkie_mock_env:UpkieMockEnv",
    )
    gym.envs.registration.register(
        id="Upkie-Spine-GroundVelocity",
        entry_point=("upkie.envs.entry_points:make_spine_ground_velocity"),
    )
    gym.envs.registration.register(
        id="Upkie-Spine-Servos",
        entry_point="upkie.envs.upkie_spine_env:UpkieSpineEnv",
    )


__all__ = [
    "register",
]
