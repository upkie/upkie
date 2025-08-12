#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

## \namespace upkie.utils.clamp
## \brief Clamping functions.

r"""!
Clamping functions.
"""

from typing import Optional

from ..logging import logger


def clamp(
    value: float, lower: Optional[float] = None, upper: Optional[float] = None
) -> float:
    r"""!
    Clamp a value between a lower and an upper bound.

    \param value Value to clamp.
    \param lower Lower bound.
    \param upper Upper bound.
    """
    if lower is not None and value < lower:
        return lower
    if upper is not None and value > upper:
        return upper
    return value


def clamp_abs(value: float, bound: float):
    r"""!
    Clamp a value's absolute value, keeping its sign.

    \param value Value to clamp.
    \param bound Absolute value bound.
    """
    return clamp(value, -bound, bound)


def clamp_and_warn(value: float, lower: float, upper: float, label: str):
    r"""!
    Clamp a value between a lower and an upper bound, warning if the value is
    changed.

    \param value Value to clamp.
    \param lower Lower bound.
    \param upper Upper bound.
    \param label Label to describe the value.
    """
    if value < lower:
        logger.warning(f"{label}={value} clamped to {lower=}")
        return lower
    elif value > upper:
        logger.warning(f"{label}={value} clamped to {upper=}")
        return upper
    return value
