#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

## \namespace upkie.utils.nested_update
## \brief Functions to work with nested dictionaries.

"""!
Functions to work with nested dictionaries.
"""

import numpy as np

from ..exceptions import UpkieRuntimeError


def nested_update(target_dict: dict, new_dict: dict) -> None:
    r"""!
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

    \param target_dict Output dictionary.
    \param new_dict Input dictionary with new values to update.
    """
    for key, value in new_dict.items():
        if (
            key in target_dict
            and isinstance(target_dict[key], dict)
            and isinstance(value, dict)
        ):
            nested_update(target_dict[key], value)
        elif (
            key in target_dict
            and key in new_dict
            and not isinstance(new_dict[key], type(target_dict[key]))
        ):
            old_type = type(target_dict[key])
            new_type = type(new_dict[key])
            if issubclass(old_type, np.ndarray):
                new_value = np.array(new_dict[key])
                target_dict[key] = new_value
            else:  # unclear how to convert
                raise UpkieRuntimeError(
                    f"key '{key}' is already present in dictionary with type "
                    f"'{old_type}', but updated value has type '{new_type}'"
                )
        else:
            target_dict[key] = new_dict[key]
