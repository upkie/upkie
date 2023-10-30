#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria
# SPDX-License-Identifier: Apache-2.0

import os

import yaml

PATH = os.path.abspath(os.path.dirname(__file__))

SPINE_CONFIG = None

with open(PATH + "/spine.yaml", "r") as fh:
    SPINE_CONFIG = yaml.safe_load(fh)
