#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2022 Stéphane Caron
# Copyright 2023 Inria
# SPDX-License-Identifier: Apache-2.0

import math
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import numpy as np
from gymnasium import spaces
from numpy.typing import NDArray

from upkie.utils.exceptions import UpkieException

from .upkie_wheeled_pendulum import UpkieWheeledPendulum


class UpkieGroundVelocity(UpkieWheeledPendulum):

    """!
    Environment where Upkie balances by ground velocity control.

    The environment id is ``UpkieGroundVelocity-v2``.

    @note To deep reinforcement learning practitioners: the observation space
    and action space are not normalized.

    ### Action space

    Vectorized actions have the following structure:

    <table>
        <tr>
            <td><strong>Index</strong></td>
            <td><strong>Description</strong></td>
            </tr>
        <tr>
            <td>``0``</td>
            <td>Ground velocity in [m] / [s].</td>
        </tr>
    </table>

    Note that, while this action is not normalized, [-1, 1] m/s is a reasonable
    range for ground velocities.

    ### Observation space

    Vectorized observations have the following structure:

    <table>
        <tr>
            <td><strong>Index</strong></td>
            <td><strong>Description</strong></td>
        </tr>
        <tr>
            <td>0</td>
            <td>Pitch angle of the base with respect to the world vertical, in
            radians. This angle is positive when the robot leans forward.</td>
        </tr>
        <tr>
            <td>1</td>
            <td>Position of the average wheel contact point, in meters.</td>
        </tr>
        <tr>
            <td>2</td>
            <td>Body angular velocity of the base frame along its lateral axis,
            in radians per seconds.</td>
        </tr>
        <tr>
            <td>3</td>
            <td>Velocity of the average wheel contact point, in meters per
            seconds.</td>
        </tr>
    </table>

    ### Attributes

    The environment class defines the following attributes:

    - ``version``: Environment version number.
    - ``wheel_radius``: Wheel radius in [m].

    """

    @dataclass
    class RewardWeights:
        position: float = 1.0
        velocity: float = 1.0

    _last_action: float
    version: int = 2
    wheel_radius: float

    def __init__(
        self,
        max_ground_velocity: float = 1.0,
        reward_weights: Optional[RewardWeights] = None,
        wheel_radius: float = 0.06,
        **kwargs,
    ):
        """!
        Initialize environment.

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

        weights: UpkieGroundVelocity.RewardWeights = (
            reward_weights
            if reward_weights is not None
            else UpkieGroundVelocity.RewardWeights()
        )

        MAX_BASE_PITCH: float = np.pi
        MAX_GROUND_POSITION: float = float("inf")
        MAX_BASE_ANGULAR_VELOCITY: float = 1000.0  # rad/s
        observation_limit = np.array(
            [
                MAX_BASE_PITCH,
                MAX_GROUND_POSITION,
                MAX_BASE_ANGULAR_VELOCITY,
                max_ground_velocity,
                max_ground_velocity,
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
            -np.float32(max_ground_velocity),
            +np.float32(max_ground_velocity),
            shape=(1,),
            dtype=np.float32,
        )

        self._last_action = 0.0
        self.reward_weights = weights
        self.wheel_radius = wheel_radius

    def reset(
        self,
        *,
        seed: Optional[int] = None,
        options: Optional[dict] = None,
    ) -> Tuple[NDArray[float], Dict]:
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
        self._last_action = 0.0
        return super().reset(seed=seed)

    def dictionarize_action(self, action: NDArray[float]) -> dict:
        """!
        Convert action vector into a spine action dictionary.

        @param action Action vector.
        @returns Action dictionary.
        """
        self._last_action = action[0]
        ground_velocity = action[0]
        wheel_velocity = ground_velocity / self.wheel_radius
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

    def vectorize_observation(self, observation_dict: dict) -> np.ndarray:
        observation = super().vectorize_observation(observation_dict)
        augmented_obs = np.hstack(
            [observation, [self._last_action]],
            dtype=np.float32,
        )
        return augmented_obs

    def get_reward(
        self, observation: NDArray[float], action: NDArray[float]
    ) -> float:
        """!
        Get reward from observation and action.

        @param observation Observation vector.
        @param action Action vector.
        @returns Reward.
        """
        pitch = observation[0]
        ground_position = observation[1]
        ground_velocity = observation[2]
        angular_velocity = observation[3]

        tip_height = 0.58  # [m]
        tip_position = ground_position + tip_height * np.sin(pitch)
        tip_velocity = (
            ground_velocity + tip_height * angular_velocity * np.cos(pitch)
        )

        sigma = 0.6  # [rad] ~= 35 [deg]
        # at pitch = 35 deg = sigma the position reward is exp(-1) ~= 0.36
        # at pitch = 45 deg ~= 1.3 * sigma the position reward is 0.16
        position_reward = np.exp(-((pitch / sigma) ** 2))

        velocity_penalty = -np.abs(ground_velocity)

        return (
            self.reward_weights.position * position_reward
            + self.reward_weights.velocity * velocity_penalty
        )
