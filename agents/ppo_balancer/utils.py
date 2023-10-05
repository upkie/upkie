#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria
# SPDX-License-Identifier: Apache-2.0


def gin_operative_config_dict(operative_config: dict) -> dict:
    output = {}
    for key, values in operative_config.items():
        name = key[1].split(".")[1]
        if name not in output:
            output[name] = {}
        for k, v in values.items():
            output[name][k] = v
    return output
