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

from .upkie_ground_velocity import UpkieGroundVelocity
from .upkie_mock_servos import UpkieMockServos
from .upkie_servo_positions import UpkieServoPositions
from .upkie_servo_torques import UpkieServoTorques
from .upkie_spine_servos import UpkieSpineServos
from .upkie_servos import UpkieServos


def register() -> None:
    """!
    Register Upkie environments with Gymnasium.
    """
    # Environments
    envs = (
        ("UpkieMockServos", UpkieMockServos),
        ("UpkieServoPositions", UpkieServoPositions),
        ("UpkieServoTorques", UpkieServoTorques),
        ("UpkieSpineServos", UpkieSpineServos),
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
    "UpkieGroundVelocity",
    "UpkieMockServos",
    "UpkieServoPositions",
    "UpkieServoTorques",
    "UpkieSpineServos",
    "UpkieServos",
    "register",
]
