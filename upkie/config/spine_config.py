#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
#
## \namespace upkie.config.spine_config
## \brief Configuration dictionary communicated to spines upon reset.

from pathlib import Path
from typing import Any, Dict

import yaml

from upkie.utils.nested_update import nested_update

from ..logging import logger


def _get_user_config_path() -> Path:
    r"""!
    Get path to the user configuration file.

    \return Path to ~/.config/upkie/config.yaml or ~/.config/upkie/config.yml,
        whichever exists (preferring .yaml).
    """
    config_dir = Path.home() / ".config" / "upkie"
    yaml_path = config_dir / "config.yaml"
    if yaml_path.exists():
        return yaml_path
    return config_dir / "config.yml"


def _load_yaml_config(config_path: Path) -> Dict[str, Any]:
    r"""!
    Load YAML configuration from file.

    \param config_path Path to the configuration file.
    \return Configuration dictionary, empty if the file doesn't exist or can't
        be loaded.
    """
    if not config_path.exists():
        return {}

    try:
        with config_path.open("r") as file:
            return yaml.safe_load(file)
    except Exception as e:
        logger.warning(f"Failed to load config from {config_path}: {e}")
        return {}


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
    "bullet": {
        "gui": True,
        "reset": {
            "orientation_base_in_world": [1.0, 0.0, 0.0, 0.0],
            "position_base_in_world": [0.0, 0.0, 0.6],
        },
        "torque_control": {
            "kp": 20.0,
            "kd": 1.0,
        },
    },
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
SPINE_CONFIG = merge_user_spine_config(
    _DEFAULT_SPINE_CONFIG,
    user_config=_load_yaml_config(_get_user_config_path()),
)
