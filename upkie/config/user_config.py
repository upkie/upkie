#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

## \namespace upkie.config.user_config
## \brief Load local configuration to a `USER_CONFIG` dictionary.

from pathlib import Path
from typing import Any, Dict, Optional

import yaml


def _get_user_config_path() -> Path:
    r"""!
    Get path to the user configuration file.

    \return Path to ~/.config/upkie/config.yml
    """
    config_dir = Path.home() / ".config" / "upkie"
    return config_dir / "config.yml"


def _load_yaml_config(config_path: Path) -> Optional[Dict[str, Any]]:
    r"""!
    Load YAML configuration from file.

    \param config_path Path to the configuration file.
    \return Configuration dictionary, or None if file doesn't exist or can't be
        loaded.
    """
    if not config_path.exists():
        return None

    try:
        with config_path.open("r") as file:
            return yaml.safe_load(file)
    except Exception as e:
        print(f"Warning: Failed to load config from {config_path}: {e}")
        return None


## This dictionary contains the complete YAML configuration loaded from the
## user's configuration file at `~/.config/upkie/config.yml`. If the file
## doesn't exist or can't be loaded, this will be an empty dict.
USER_CONFIG = _load_yaml_config(_get_user_config_path()) or {}
