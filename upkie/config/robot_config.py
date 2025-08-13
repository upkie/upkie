#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2025 Inria

## \namespace upkie.config.robot_config
## \brief Configuration dictionary for robot parameters.

from typing import Any, Dict

from upkie.utils.nested_update import nested_update

from .user_config import USER_CONFIG


def _merge_user_robot_config(
    default_config: Dict[str, Any],
    user_config: Dict[str, Any],
) -> Dict[str, Any]:
    r"""!
    Merge user configuration with default robot configuration.

    \param default_config Default robot configuration dictionary.
    \param user_config User configuration dictionary.
    \return Merged configuration with user overrides applied.
    """
    merged_config = default_config.copy()
    robot_overrides = user_config.get("robot", {})
    if robot_overrides:
        nested_update(merged_config, robot_overrides)
    return merged_config


## Default robot configuration dictionary.
_DEFAULT_ROBOT_CONFIG = {
    "leg_length": 0.58,  # m
    "mass": 5.34,  # kg
    "wheel_radius": 0.06,  # m
}

## Robot configuration dictionary with default values and user overrides.
## This dictionary will merge the default configuration with user overrides
## from ~/.config/upkie/config.yml
ROBOT_CONFIG = _merge_user_robot_config(_DEFAULT_ROBOT_CONFIG, USER_CONFIG)
