#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

import logging

import gymnasium as gym

from .upkie_base_env import UpkieBaseEnv

__all__ = [
    "UpkieBaseEnv",
    "register",
]

__envs__ = {}

try:
    from .upkie_ground_velocity import UpkieGroundVelocity

    __all__.append("UpkieGroundVelocity")
    __envs__["UpkieGroundVelocity"] = UpkieGroundVelocity
except ImportError as exn:
    __envs__["UpkieGroundVelocity"] = exn

try:
    from .upkie_servos import UpkieServos

    __all__.append("UpkieServos")
    __envs__["UpkieServos"] = UpkieServos
except ImportError as exn:
    __envs__["UpkieServos"] = exn


def register() -> None:
    """!
    Register Upkie environments with Gymnasium.
    """
    for env_name, env_class in __envs__.items():
        if isinstance(env_class, ModuleNotFoundError):
            import_error = str(env_class)
            logging.warning(
                "Cannot register %s due to missing dependency: %s",
                env_name,
                import_error,
            )
        else:  # valid gym.Env subclass
            gym.envs.registration.register(
                id=f"{env_name}-v{env_class.version}",
                entry_point=f"upkie.envs:{env_name}",
            )
