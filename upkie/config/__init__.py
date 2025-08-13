#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

## \namespace upkie.config
## \brief Static robot configuration.

from .bullet_config import BULLET_CONFIG
from .spine_config import SPINE_CONFIG, merge_user_spine_config
from .user_config import USER_CONFIG, get_user_config

__all__ = [
    "BULLET_CONFIG",
    "SPINE_CONFIG",
    "USER_CONFIG",
    "get_user_config",
    "merge_user_spine_config",
]
