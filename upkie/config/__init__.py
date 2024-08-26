#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

## \namespace upkie.config
## \brief Static robot configuration.

import yaml

PATH = os.path.abspath(os.path.dirname(__file__))

SPINE_CONFIG = None

with open(PATH + "/spine.yaml", "r") as fh:
    SPINE_CONFIG = yaml.safe_load(fh)
