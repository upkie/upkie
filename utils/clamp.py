#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2022 Stéphane Caron
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from typing import Optional


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
