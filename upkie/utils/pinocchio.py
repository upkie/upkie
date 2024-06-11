#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

from typing import Tuple

import numpy as np
import pinocchio as pin
from numpy.typing import NDArray


def box_position_limits(
    model: pin.Model,
) -> Tuple[NDArray[float], NDArray[float]]:
    r"""!
    Compute position limits in box format:

    \f[
    q_{min} \leq q \leq q_{max}
    \f]

    \param model Pinocchio model.
    @returns Tuple ``(q_min, q_max)`` of lower and upper position limits, with
    -infinity and +infinity where there is no limit.
    """
    no_position_limit = np.logical_or(
        model.upperPositionLimit > 1e20,
        model.upperPositionLimit < model.lowerPositionLimit + 1e-10,
    )
    q_min = model.lowerPositionLimit.copy()
    q_max = model.upperPositionLimit.copy()
    q_min[no_position_limit] = -np.inf
    q_max[no_position_limit] = +np.inf
    return q_min, q_max


def box_velocity_limits(
    model: pin.Model,
) -> Tuple[NDArray[float], NDArray[float]]:
    r"""!
    Compute velocity limits in box format:

    \f[
    v_{min} \leq v \leq v_{max}
    \f]

    \param model Pinocchio model.
    \return Velocity limits, with -infinity and +infinity where there is no
    limit.
    """
    no_velocity_limit = np.logical_or(
        model.velocityLimit > 1e20,
        model.velocityLimit < 1e-10,
    )
    v_max = model.velocityLimit.copy()
    v_max[no_velocity_limit] = np.inf
    return v_max


def box_torque_limits(
    model: pin.Model,
) -> Tuple[NDArray[float], NDArray[float]]:
    r"""!
    Compute velocity limits in box format:

    \f[
    \tau_{min} \leq \tau \leq \tau_{max}
    \f]

    \param model Pinocchio model.
    \return Torque limits, with -infinity and +infinity where there is no
    limit.
    """
    no_torque_limit = np.logical_or(
        model.effortLimit > 1e20,
        model.effortLimit < 1e-10,
    )
    tau_max = model.effortLimit.copy()
    tau_max[no_torque_limit] = np.inf
    return tau_max
