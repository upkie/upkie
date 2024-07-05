#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

__UPPER_LEG_JOINTS = (
    f"{side}_{joint}"
    for side in ("left", "right")
    for joint in ("hip", "knee")
)


def upper_leg_joints():
    return __UPPER_LEG_JOINTS
