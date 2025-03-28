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
from .upkie_servo_positions import UpkieServoPositions
from .upkie_servo_torques import UpkieServoTorques
from .upkie_servos import UpkieServos


def make_upkie_ground_velocity(**kwargs):
    UPKIE_SERVOS_KWARGS = (
        "frequency",
        "frequency_checks",
        "init_state",
        "regulate_frequency",
        "shm_name",
        "spine_config",
    )
    servos_kwargs = {
        key: value
        for key, value in kwargs.items()
        if key in UPKIE_SERVOS_KWARGS
    }
    ground_velocity_kwargs = {
        key: value
        for key, value in kwargs.items()
        if key not in UPKIE_SERVOS_KWARGS
    }
    env = UpkieServos(**servos_kwargs)
    return UpkieGroundVelocity(env, **ground_velocity_kwargs)


def register() -> None:
    """!
    Register Upkie environments with Gymnasium.
    """
    # Environments
    envs = (
        ("UpkieServoPositions", UpkieServoPositions),
        ("UpkieServoTorques", UpkieServoTorques),
        ("UpkieServos", UpkieServos),
    )
    for env_name, env_class in envs:
        gym.envs.registration.register(
            id=f"{env_name}-v{env_class.version}",
            entry_point=f"upkie.envs:{env_name}",
        )

    # Wrappers
    gym.envs.registration.register(
        id=f"UpkieGroundVelocity-v{UpkieGroundVelocity.version}",
        entry_point="upkie.envs:make_upkie_ground_velocity",
    )


__all__ = [
    "UpkieGroundVelocity",
    "UpkieServoPositions",
    "UpkieServoTorques",
    "UpkieServos",
    "register",
]
