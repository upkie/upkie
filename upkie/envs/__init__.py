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

from .upkie_env import UpkieEnv
from .upkie_pendulum import UpkiePendulum
from .upkie_servos import UpkieServos


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
            for action in ["servos", "pendulum"]:
                action_cap = action.capitalize()
                gym.envs.registration.register(
                    id=f"{robot_cap}-{backend_cap}-{action_cap}",
                    entry_point=(
                        f"upkie.envs.entry_points:make_"
                        f"{robot}_{backend}_{action}"
                    ),
                )


__all__ = [
    "UpkieEnv",
    "UpkiePendulum",
    "UpkieServos",
    "register",
]
