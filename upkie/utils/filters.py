#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

from typing import Tuple

from .clamp import clamp


def abs_bounded_derivative_filter(
    prev_output: float,
    new_input: float,
    dt: float,
    max_output: float,
    max_derivative: float,
) -> float:
    r"""!
    Filter signal so that the absolute values of its output and output
    derivative stay within bounds.

    \param prev_output Previous filter output, or initial value.
    \param new_input New filter input.
    \param dt Sampling period in [s].
    \param max_output Maximum absolute value of the output.
    \param max_derivative Maximum absolute value of the output derivative.
    \return New filter output.
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
    r"""!
    Filter signal so that its output and output derivative stay within bounds.

    \param prev_output Previous filter output, or initial value.
    \param new_input New filter input.
    \param dt Sampling period in [s].
    \param output_bounds Min and max value for the output.
    \param derivative_bounds Min and max value for the output derivative.
    \return New filter output.
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
    r"""!
    Low-pass filter.

    \param prev_output Previous filter output, or initial value.
    \param cutoff_period Time constant of the filter in [s].
    \param new_input New filter input.
    \param dt Sampling period in [s].
    \return New filter output.
    """
    alpha = dt / cutoff_period
    assert alpha < 0.5  # Nyquist-Shannon sampling theorem
    return prev_output + alpha * (new_input - prev_output)
