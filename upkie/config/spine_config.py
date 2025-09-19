#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

## \namespace upkie.config.spine_config
## \brief Configuration dictionary communicated to spines upon reset.

from typing import Any, Dict

from upkie.utils.nested_update import nested_update

from .bullet_config import BULLET_CONFIG
from .user_config import USER_CONFIG


def merge_user_spine_config(
    default_config: Dict[str, Any],
    user_config: Dict[str, Any],
) -> Dict[str, Any]:
    r"""!
    Merge user configuration with default spine configuration.

    \param default_config Default spine configuration dictionary.
    \param user_config User configuration dictionary.
    \return Merged configuration with user overrides applied.
    """
    merged_config = default_config.copy()
    spine_overrides = user_config.get("spine", {})
    if spine_overrides:
        nested_update(merged_config, spine_overrides)
    return merged_config


## Default spine configuration dictionary with baseline settings.
## This dictionary contains the default configuration values for various
## spine components including bullet physics, contact detection, and odometry.
_DEFAULT_SPINE_CONFIG = {
    "bullet": BULLET_CONFIG,
    "floor_contact": {
        "upper_leg_torque_threshold": 10.0,
    },
    "wheel_contact": {
        "cutoff_period": 0.2,
        "liftoff_inertia": 0.001,
        "min_touchdown_acceleration": 2.0,
        "min_touchdown_torque": 0.015,
        "touchdown_inertia": 0.004,
    },
    "wheel_odometry": {
        "signed_radius": {
            "left_wheel": +0.05,
            "right_wheel": -0.05,
        }
    },
}

## Spine configuration dictionary used as defaults by Gymnasium environments.
## This dictionary will merge the default configuration with user overrides
## from ~/.config/upkie/config.yml
SPINE_CONFIG = merge_user_spine_config(_DEFAULT_SPINE_CONFIG, USER_CONFIG)
