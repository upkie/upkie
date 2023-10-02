#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria
# SPDX-License-Identifier: Apache-2.0


def nested_update(target_dict: dict, new_dict: dict) -> None:
    """!
    Update a target dictionary recursively.

    Consider the following example:

        >>> d = {"a": {"b": 1}}
        >>> d2 = {"a": {"c": 2}}
        >>> d.update(d2)
        >>> d
        {'a': {'c': 2}}

    With this function:

        >>> d = {"a": {"b": 1}}
        >>> d2 = {"a": {"c": 2}}
        >>> d.update(d2)
        >>> d
        {'a': {'b': 1, 'c': 2}}

    @param target_dict Output dictionary.
    @param new_dict Input dictionary with new values to update.
    """
    for key, value in new_dict.items():
        if (
            key in target_dict
            and isinstance(target_dict[key], dict)
            and isinstance(value, dict)
        ):
            nested_update(target_dict[key], value)
        else:
            target_dict[key] = new_dict[key]
