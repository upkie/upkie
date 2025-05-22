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

from .upkie_servos_mock import UpkieServosMock
from .upkie_servos_spine import UpkieServosSpine


def register() -> None:
    """!
    Register Upkie environments with Gymnasium.
    """
    # Environments
    envs = (
        ("Upkie-Servos-Mock", UpkieServosMock),
        ("Upkie-Servos-Spine", UpkieServosSpine),
    )
    for env_name, env_class in envs:
        gym.envs.registration.register(
            id=env_name,
            entry_point=f"upkie.envs:{env_name}",
        )

    # Wrappers
    gym.envs.registration.register(
        id="UpkieGroundVelocity",
        entry_point=(
            "upkie.envs.upkie_ground_velocity:make_upkie_ground_velocity"
        ),
    )


__all__ = [
    "register",
]
