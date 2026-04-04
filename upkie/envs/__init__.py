#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
#
## \namespace upkie.envs
## \brief Gymnasium environments.
##
## See the [list of environments](\ref gym-environments) for more details.

import gymnasium as gym

from .upkie_base_velocity import UpkieBaseVelocity
from .upkie_env import UpkieEnv
from .upkie_gyropod import UpkieGyropod
from .upkie_pendulum import UpkiePendulum
from .upkie_servos import UpkieServos

_ACTION_NAMES = {
    "servos": "Servos",
    "pendulum": "Pendulum",
    "gyropod": "Gyropod",
    "base_velocity": "BaseVelocity",
}


def register() -> None:
    """!
    Register Upkie environments with Gymnasium.

    Environments are named as `Upkie-<Backend>-<ActionSpace>`.
    """
    for robot in ["upkie", "cookie"]:
        robot_cap = robot.capitalize()
        for backend in ["genesis", "mock", "pybullet", "spine"]:
            backend_cap = (
                "PyBullet" if backend == "pybullet" else backend.capitalize()
            )
            for action, action_cap in _ACTION_NAMES.items():
                action_cap = _ACTION_NAMES[action]
                gym.envs.registration.register(
                    id=f"{robot_cap}-{backend_cap}-{action_cap}",
                    entry_point=(
                        f"upkie.envs.entry_points:make_"
                        f"{robot}_{backend}_{action}"
                    ),
                )


__all__ = [
    "UpkieBaseVelocity",
    "UpkieEnv",
    "UpkieGyropod",
    "UpkiePendulum",
    "UpkieServos",
    "register",
]
