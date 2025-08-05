#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2025 Inria

## \var BULLET_CONFIG
## Bullet simulation configuration dictionary.
BULLET_CONFIG = {
    "follower_camera": False,
    "gui": True,
    "reset": {
        "orientation_base_in_world": [1.0, 0.0, 0.0, 0.0],
        "position_base_in_world": [0.0, 0.0, 0.6],
    },
    "torque_control": {
        "kp": 20.0,
        "kd": 1.0,
    },
}
