#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

## \namespace upkie.config
## \brief Static robot configuration.

from .bullet_config import BULLET_CONFIG
from .spine_config import SPINE_CONFIG

__all__ = [
    "BULLET_CONFIG",
    "SPINE_CONFIG",
]
