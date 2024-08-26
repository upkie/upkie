#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

## \namespace upkie.config
## \brief Static robot configuration.

from os.path import abspath, dirname

import yaml

## \var SPINE_CONFIG
## Spine configuration dictionary used as defaults by Gym environments.
SPINE_CONFIG = None

with open(abspath(dirname(__file__)) + "/spine.yaml", "r") as fh:
    SPINE_CONFIG = yaml.safe_load(fh)
