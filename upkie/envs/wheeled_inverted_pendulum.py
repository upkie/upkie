#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria
# SPDX-License-Identifier: Apache-2.0

"""Wheeled inverted pendulum."""

from typing import Optional, Tuple

import gymnasium
import numpy as np
from gymnasium import spaces
from numpy.typing import NDArray

from upkie.utils.spdlog import logging

GRAVITY: float = 9.81  # [m] / [s]²


def integrate_accel(x, v, a, dt):
    x2 = x + dt * (v + dt * (a / 2))
    v2 = v + dt * a
    return x2, v2


class WheeledInvertedPendulum(gymnasium.Env):
    r"""!
    Wheeled inverted pendulum model.

    This environment has the same observation and action spaces as \ref
    upkie.envs.upkie_ground_velocity.UpkieGroundVelocity. Model assumptions and
    discretization are summed up in [this
    note](https://scaron.info/robotics/wheeled-inverted-pendulum-model.html).
    """

    ## @var action_space
    ## Action space.
    action_space: spaces.box.Box

    ## @var dt
    ## Period of the control loop in seconds.
    dt: float

    ## @var observation_space
    ## Observation space.
    observation_space: spaces.box.Box

    def __init__(
        self,
        frequency: float = 200.0,
        length: float = 0.6,
        max_ground_accel: float = 10.0,
        max_ground_velocity: float = 1.0,
    ):
        r"""!
        Initialize a new environment.

        \param[in] frequency Regulated frequency of the control loop, in Hz.
        \param[in] length Length of the pole.
        \param max_ground_velocity Maximum commanded ground velocity in [m] /
            [s].
        \param[in] max_ground_accel  Maximum acceleration of the ground point,
            in [m] / [s]².
        """
        # gymnasium.Env: observation_space
        MAX_BASE_PITCH: float = np.pi
        MAX_GROUND_POSITION: float = float("inf")
        MAX_BASE_ANGULAR_VELOCITY: float = 1000.0  # rad/s
        observation_limit = np.array(
            [
                MAX_BASE_PITCH,
                MAX_GROUND_POSITION,
                MAX_BASE_ANGULAR_VELOCITY,
                max_ground_velocity,
            ],
            dtype=float,
        )
        self.observation_space = spaces.Box(
            -observation_limit,
            +observation_limit,
            shape=observation_limit.shape,
            dtype=observation_limit.dtype,
        )

        # gymnasium.Env: action_space
        action_limit = np.array([max_ground_velocity], dtype=float)
        self.action_space = spaces.Box(
            -action_limit,
            +action_limit,
            shape=action_limit.shape,
            dtype=action_limit.dtype,
        )

        self.dt = 1.0 / self.__frequency
        self.__max_ground_accel = max_ground_accel
        self.__omega = np.sqrt(GRAVITY / length)
        self.__state = np.zeros(4)

    def reset(
        self,
        *,
        seed: Optional[int] = None,
        options: Optional[dict] = None,
    ) -> Tuple[NDArray[float], dict]:
        r"""!
        Resets the spine and get an initial observation.

        \param seed Number used to initialize the environment’s internal random
            number generator.
        \param options Currently unused.
        \return
            - ``observation``: Initial vectorized observation, i.e. an element
              of the environment's ``observation_space``.
            - ``info``: Dictionary with auxiliary diagnostic information. For
              Upkie this is the full observation dictionary sent by the spine.
        """
        super().reset(seed=seed)
        observation = self.__state
        info = {}
        return observation, info

    def step(
        self,
        action: NDArray[float],
    ) -> Tuple[NDArray[float], float, bool, bool, dict]:
        r"""!
        Run one timestep of the environment's dynamics. When the end of the
        episode is reached, you are responsible for calling `reset()` to reset
        the environment's state.

        \param action Action from the agent.
        \return
            - ``observation``: Observation of the environment, i.e. an element
              of its ``observation_space``.
            - ``reward``: Reward returned after taking the action.
            - ``terminated``: Whether the agent reached a terminal state,
              which can be a good or a bad thing. When true, the user needs to
              call :func:`reset()`.
            - ``truncated'': Whether the episode is reaching max number of
              steps. This boolean can signal a premature end of the episode,
              i.e. before a terminal state is reached. When true, the user
              needs to call :func:`reset()`.
            - ``info``: Dictionary with auxiliary diagnostic information. For
              us this is the full observation dictionary coming from the spine.
        """
        theta_0, r_0, thetad_0, rd_0 = self.__state
        rdd_0 = action[0]
        thetadd_0 = self.__omega**2 * (
            np.sin(theta_0) - (rdd_0 / GRAVITY) * np.cos(theta_0)
        )

        r, rd = integrate_accel(r_0, rd_0, rdd_0, self.dt)
        theta, thetad = integrate_accel(theta_0, thetad_0, thetadd_0, self.dt)
        self.__state = np.array([r, theta, rd, thetad]).flatten()

        observation = self.__state
        reward = self.get_reward(observation, action)
        terminated = self.detect_fall(theta)
        truncated = False
        info = {}
        return observation, reward, terminated, truncated, info

    def detect_fall(self, pitch: float) -> bool:
        r"""!
        Detect a fall based on the base-to-world pitch angle.

        \param pitch Pitch angle of the rotation from base to world frame.
        \return True if and only if a fall is detected.
        """
        if abs(pitch) > self.fall_pitch:
            logging.warning(
                "Fall detected (pitch=%.2f rad, fall_pitch=%.2f rad)",
                abs(pitch),
                self.fall_pitch,
            )
            return True
        return False
