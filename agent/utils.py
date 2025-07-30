#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron
# Copyright 2024 Inria

from typing import Tuple, TypeVar

import numpy as np

T = TypeVar("T", float, np.ndarray)


def abs_bounded_derivative_filter(
    prev_output: T,
    new_input: T,
    dt: float,
    max_derivative: T,
) -> T:
    r"""!
    Filter signal so that the absolute values of its output and output
    derivative stay within bounds.

    \param prev_output Previous filter output, or initial value.
    \param new_input New filter input.
    \param dt Sampling period in [s].
    \param max_derivative Maximum absolute value of the output derivative.
    \return New filter output.
    """
    return bounded_derivative_filter(
        prev_output,
        new_input,
        dt,
        (-max_derivative, max_derivative),
    )


def bounded_derivative_filter(
    prev_output: T,
    new_input: T,
    dt: float,
    derivative_bounds: Tuple[T, T],
) -> T:
    r"""!
    Filter signal so that its output and output derivative stay within bounds.

    \param prev_output Previous filter output, or initial value.
    \param new_input New filter input.
    \param dt Sampling period in [s].
    \param derivative_bounds Min and max value for the output derivative.
    \return New filter output.
    """
    derivative = (new_input - prev_output) / dt
    derivative = np.clip(derivative, *derivative_bounds)
    output = prev_output + derivative * dt
    return output
