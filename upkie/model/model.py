#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria


class Model:
    """!
    Structural parameters of Upkie wheeled bipeds.
    """

    def __init__(self):
        """!
        Initialize model with default parameters.
        """
        self.__leg_joints = [
            f"{side}_{joint}"
            for side in ("left", "right")
            for joint in ("hip", "knee")
        ]

    @property
    def leg_joints(self):
        """!
        Get the list of leg (i.e., non-wheel) joints.
        """
        return self.__leg_joints
