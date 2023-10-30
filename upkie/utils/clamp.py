#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2022 Stéphane Caron
# SPDX-License-Identifier: Apache-2.0

from typing import Optional

from .spdlog import logging


def clamp(
    value: float, lower: Optional[float] = None, upper: Optional[float] = None
) -> float:
    """
    Clamp a value between a lower and an upper bound.

    Args:
        value: Value to clamp.
        lower: Lower bound.
        upper: Upper bound.
    """
    if lower is not None and value < lower:
        return lower
    if upper is not None and value > upper:
        return upper
    return value


def clamp_abs(value: float, bound: float):
    """
    Clamp a value's absolute value, keeping its sign.

    Args:
        value: Value to clamp.
        bound: Absolute value bound.
    """
    return clamp(value, -bound, bound)


def clamp_and_warn(value: float, lower: float, upper: float, label: str):
    """
    Clamp a value between a lower and an upper bound, warning if the value is
    changed.

    Args:
        value: Value to clamp.
        lower: Lower bound.
        upper: Upper bound.
        label: Label to describe the value.
    """
    if value < lower:
        logging.warning(f"{label}={value} clamped to {lower=}")
        return lower
    elif value > upper:
        logging.warning(f"{label}={value} clamped to {upper=}")
        return upper
    return value
