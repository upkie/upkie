#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

## \var SPINE_CONFIG
## Spine configuration dictionary used as defaults by Gymnasium environments.
SPINE_CONFIG = {
    "bullet": {
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
        "monitor": {
            "contacts": {
                "wheels":{
                    "left_wheel_tire": True,
                    "right_wheel_tire": True,
                },
                "envs":{
                    "exclude": {
                        "left_wheel_tire": True,
                        "right_wheel_tire": True,
                        },
                }
            }
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
