#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

import abc

import gin


@gin.configurable
class SagittalBalancer(abc.ABC):
    """Base class for sagittal balancers."""

    def __init__(
        self,
        fall_pitch: float,
        max_ground_accel: float,
        max_ground_velocity: float,
    ):
        """Initialize balancer.

        Args:
            fall_pitch: Fall pitch threshold, in radians.
            max_ground_accel: Maximum commanded ground acceleration no matter
                what, in [m] / [s]².
            max_ground_velocity: Maximum commanded ground velocity no matter
                what, in [m] / [s].
        """
        self.fall_pitch = fall_pitch
        self.max_ground_accel = max_ground_accel
        self.max_ground_velocity = max_ground_velocity

    @abc.abstractmethod
    def compute_ground_velocity(
        self,
        target_ground_velocity: float,
        observation: dict,
        dt: float,
    ) -> float:
        """Compute a new ground velocity.

        Args:
            target_ground_velocity: Target ground velocity in [m] / [s].
            observation: Latest observation dictionary.
            dt: Time in [s] until next cycle.

        Returns:
            New ground velocity, in [m] / [s].
        """
