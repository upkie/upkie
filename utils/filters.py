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

from typing import Tuple

from utils.clamp import clamp


def abs_bounded_derivative_filter(
    prev_output: float,
    new_input: float,
    dt: float,
    max_output: float,
    max_derivative: float,
) -> float:
    """
    Filter signal so that the absolute values of its output and output
    derivative stay within bounds.

    Args:
        prev_output: Previous filter output, or initial value.
        new_input: New filter input.
        dt: Sampling period in [s].
        max_output: Maximum absolute value of the output.
        max_derivative: Maximum absolute value of the output derivative.

    Returns:
        New filter output.
    """
    return bounded_derivative_filter(
        prev_output,
        new_input,
        dt,
        (-max_output, max_output),
        (-max_derivative, max_derivative),
    )


def bounded_derivative_filter(
    prev_output: float,
    new_input: float,
    dt: float,
    output_bounds: Tuple[float, float],
    derivative_bounds: Tuple[float, float],
) -> float:
    """
    Filter signal so that its output and output derivative stay within bounds.

    Args:
        prev_output: Previous filter output, or initial value.
        new_input: New filter input.
        dt: Sampling period in [s].
        output_bounds: Minimum and maximum value for the output.
        derivative_bounds: Minimum and maximum value for the output derivative.

    Returns:
        New filter output.
    """
    derivative = (new_input - prev_output) / dt
    derivative = clamp(derivative, *derivative_bounds)
    output = prev_output + derivative * dt
    return clamp(output, *output_bounds)


def low_pass_filter(
    prev_output: float,
    cutoff_period: float,
    new_input: float,
    dt: float,
) -> float:
    """
    Low-pass filter.

    Args:
        prev_output: Previous filter output, or initial value.
        cutoff_period: Time constant of the filter in [s].
        new_input: New filter input.
        dt: Sampling period in [s].

    Returns:
        New filter output.
    """
    alpha = dt / cutoff_period
    assert alpha < 0.5  # Nyquist-Shannon sampling theorem
    return prev_output + alpha * (new_input - prev_output)
