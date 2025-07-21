#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

"""Python module to control Upkie wheeled bipeds."""

from . import config, envs, model, spine, utils

__version__ = "8.1.1"

__all__ = [
    "config",
    "envs",
    "model",
    "spine",
    "utils",
]
