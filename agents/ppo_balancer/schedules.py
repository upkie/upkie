#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria
# SPDX-License-Identifier: Apache-2.0


from typing import Callable


def constant_schedule(param: float) -> Callable[[float], float]:
    def schedule(x: float) -> float:
        return param

    return schedule


def linear_schedule(param: float) -> Callable[[float], float]:
    """!
    Linear schedule for a function argument between 0 and 1.

    @param param Maximum output value (when parameter is 1).
    @return Corresponding linear function.
    """

    def schedule(x: float) -> float:
        """!
        Compute the current learning rate from remaining progress.

        @param x Progress decreasing from 1 (beginning) to 0.
        @return Corresponding learning rate>
        """
        return x * param

    return schedule


def exponential_decay_schedule(
    initial_value: float,
    nb_steps: int,
    factor: float = 0.1,
) -> Callable[[float], float]:
    """!
    Step-by-step exponential-decay learning rate schedule.

    @param initial_value Learning rate at the beginning of training.
    @param nb_steps Number of decay steps.
    @param factor Initial value is multiplied by this factor at each step.
    @return Function computing the current learning rate from remaining
        progress.
    """

    def schedule(progress_remaining: float) -> float:
        """!
        Compute the current learning rate from remaining progress.

        @param progress_remaining Progress decreasing from 1 (beginning) to 0.
        @return Corresponding learning rate>
        """
        step_number = int(nb_steps * (1.0 - progress_remaining))
        return initial_value * factor**step_number

    return schedule
