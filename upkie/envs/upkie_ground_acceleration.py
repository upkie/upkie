#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2022 Stéphane Caron
# Copyright 2023 Inria
# SPDX-License-Identifier: Apache-2.0

import math
from typing import Dict, Optional, Tuple

import numpy as np
from gymnasium import spaces

from upkie.envs import UpkieWheeledPendulum
from upkie.utils.clamp import clamp_abs
from upkie.utils.exceptions import UpkieException


class UpkieGroundAcceleration(UpkieWheeledPendulum):

    """!
    Environment where Upkie balances by ground acceleration control.
    """

    __ground_velocity: float
    max_ground_accel: float
    max_ground_velocity: float
    version: int = 1
    wheel_radius: float

    def __init__(
        self,
        max_ground_accel: float = 10.0,
        max_ground_velocity: float = 1.0,
        wheel_radius: float = 0.06,
        **kwargs,
    ):
        """!
        Initialize environment.

        @param max_ground_accel Maximum commanded ground acceleration in m/s^2.
        @param max_ground_velocity Maximum commanded ground velocity in m/s.
        @param wheel_radius Wheel radius in [m].
        @param kwargs Other keyword arguments are forwarded as-is to parent
            class constructors. Follow the chain up from @ref
            envs.upkie_wheeled_pendulum.UpkieWheeledPendulum
            "UpkieWheeledPendulum" for their documentation.
        """
        super().__init__(**kwargs)

        if self.dt is None:
            raise UpkieException("This environment needs a loop frequency")

        MAX_BASE_PITCH: float = np.pi
        MAX_GROUND_POSITION: float = float("inf")
        MAX_GROUND_VELOCITY: float = 2.0  # m/s
        MAX_BASE_ANGULAR_VELOCITY: float = 100.0  # rad/s
        observation_limit = np.array(
            [
                MAX_BASE_PITCH,
                MAX_GROUND_POSITION,
                MAX_BASE_ANGULAR_VELOCITY,
                MAX_GROUND_VELOCITY,
                MAX_GROUND_VELOCITY,
            ],
            dtype=np.float32,
        )

        # gymnasium.Env: observation_space
        self.observation_space = spaces.Box(
            -observation_limit,
            +observation_limit,
            shape=(5,),
            dtype=np.float32,
        )

        # gymnasium.Env: action_space
        self.action_space = spaces.Box(
            -np.float32(max_ground_accel),
            +np.float32(max_ground_accel),
            shape=(1,),
            dtype=np.float32,
        )

        self.__ground_velocity = 0.0
        self.max_ground_velocity = max_ground_velocity
        self.wheel_radius = wheel_radius

    def reset(
        self,
        *,
        seed: Optional[int] = None,
        options: Optional[dict] = None,
    ) -> Tuple[np.ndarray, Dict]:
        """!
        Resets the environment and get an initial observation.

        @param seed Number used to initialize the environment’s internal random
            number generator.
        @param options Currently unused.
        @returns
            - ``observation``: Initial vectorized observation, i.e. an element
              of the environment's ``observation_space``.
            - ``info``: Dictionary with auxiliary diagnostic information. For
              Upkie this is the full observation dictionary sent by the spine.
        """
        self.__ground_velocity = 0.0
        observation, info = super().reset(seed=seed)
        return observation, info

    def vectorize_observation(self, observation_dict: dict) -> np.ndarray:
        observation = super().vectorize_observation(observation_dict)
        augmented_obs = np.hstack(
            [observation, [self.__ground_velocity]],
            dtype=np.float32,
        )
        return augmented_obs

    def dictionarize_action(self, action: np.ndarray) -> dict:
        """!
        Convert action vector into a spine action dictionary.

        @param action Action vector.
        @returns Action dictionary.
        """
        self.__ground_acceleration = action[0]
        assert (
            abs(self.__ground_acceleration) <= self.max_ground_accel
        )  # TODO(scaron): remove

        self.__ground_velocity = clamp_abs(
            self.__ground_velocity + self.__ground_acceleration * self.dt,
            self.max_ground_velocity,
        )

        wheel_velocity = self.__ground_velocity / self.wheel_radius
        servo_dict = self.get_leg_servo_action()
        servo_dict.update(
            {
                "left_wheel": {
                    "position": math.nan,
                    "velocity": +wheel_velocity,
                },
                "right_wheel": {
                    "position": math.nan,
                    "velocity": -wheel_velocity,
                },
            }
        )
        action_dict = {"servo": servo_dict}
        return action_dict
