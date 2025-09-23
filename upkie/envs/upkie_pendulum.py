#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
#
## \namespace upkie.envs.upkie_pendulum
## \brief Backward-compatible pendulum wrapper (no yaw control).

from typing import Dict, Optional, Tuple

import gymnasium as gym
import numpy as np

from upkie.envs.upkie_gyropod import UpkieGyropod
from upkie.envs.upkie_servos import UpkieServos

## Indices into the 6D gyropod observation to extract the 4D pendulum
## observation in the original order: [pitch, ground_position,
## pitch_velocity, ground_velocity].
_PENDULUM_OBS_INDICES = [1, 0, 4, 3]


class UpkiePendulum(gym.Wrapper):
    r"""!
    Backward-compatible wrapper around UpkieGyropod with 4D obs / 1D action.

    \anchor upkie_pendulum_description

    This wrapper strips the yaw-related dimensions from \ref
    upkie.envs.upkie_gyropod.UpkieGyropod, providing the same interface as the
    former pendulum environment:

    ### Action space

    \f[
    a = \begin{bmatrix} \dot{p}^* \end{bmatrix}
    \f]

    where \f$\dot{p}^*\f$ is the commanded ground velocity in m/s.

    ### Observation space

    \f[
    o = \begin{bmatrix} \theta \\ p \\ \dot{\theta} \\ \dot{p} \end{bmatrix}
    \f]

    where we denote by:

    - \f$\theta\f$ the pitch angle of the base with respect to the world
      vertical, in radians.
    - \f$p\f$ the position of the average wheel contact point, in meters.
    - \f$\dot{\theta}\f$ the body angular velocity of the base frame along its
      lateral axis, in radians per seconds.
    - \f$\dot{p}\f$ the velocity of the average wheel contact point, in meters
      per seconds.

    As with all Upkie environments, full observations from the spine (detailed
    in \ref observations) are also available in the `info` dictionary
    returned by the reset and step functions.
    """

    ## \var action_space
    ## Action space.
    action_space: gym.spaces.Box

    ## \var observation_space
    ## Observation space.
    observation_space: gym.spaces.Box

    def __init__(
        self,
        servos_env: UpkieServos,
        fall_pitch: float = 1.0,
        max_ground_velocity: float = 3.0,
    ):
        r"""!
        Initialize environment.

        \param servos_env Upkie environment to command servomotors.
        \param fall_pitch Fall detection pitch angle, in radians.
        \param max_ground_velocity Maximum commanded ground velocity in m/s.
        """
        gyropod_env = UpkieGyropod(
            servos_env,
            fall_pitch=fall_pitch,
            max_ground_velocity=max_ground_velocity,
        )
        super().__init__(gyropod_env)

        gyropod_obs_high = gyropod_env.observation_space.high
        obs_limit = gyropod_obs_high[_PENDULUM_OBS_INDICES]
        self.observation_space = gym.spaces.Box(
            -obs_limit,
            +obs_limit,
            shape=obs_limit.shape,
            dtype=np.float32,
        )

        action_limit = np.array([max_ground_velocity], dtype=np.float32)
        self.action_space = gym.spaces.Box(
            -action_limit,
            +action_limit,
            shape=action_limit.shape,
            dtype=np.float32,
        )

        # Instance attributes
        self.env = gyropod_env.unwrapped
        self.fall_pitch = fall_pitch

    def reset(
        self,
        *,
        seed: Optional[int] = None,
        options: Optional[dict] = None,
    ) -> Tuple[np.ndarray, Dict]:
        r"""!
        Resets the environment and get an initial observation.

        \param seed Number used to initialize the environment's internal random
            number generator.
        \param options Currently unused.
        \return
            - `observation`: Initial vectorized observation, i.e. an element
              of the environment's `observation_space`.
            - `info`: Dictionary with auxiliary diagnostic information.
        """
        obs_6d, info = self.env.reset(seed=seed, options=options)
        return obs_6d[_PENDULUM_OBS_INDICES].copy(), info

    def step(
        self,
        action: np.ndarray,
    ) -> Tuple[np.ndarray, float, bool, bool, dict]:
        r"""!
        Run one timestep of the environment's dynamics.

        \param action Action from the agent: [ground_velocity].
        \return
            - `observation`: Observation of the environment.
            - `reward`: Reward returned after taking the action.
            - `terminated`: Whether the agent reached a terminal state.
            - `truncated`: Whether the episode is reaching max number of steps.
            - `info`: Dictionary with additional information.
        """
        action_2d = np.array([action[0], 0.0], dtype=np.float32)
        obs_6d, reward, terminated, truncated, info = self.env.step(action_2d)
        obs = obs_6d[_PENDULUM_OBS_INDICES].copy()
        return obs, reward, terminated, truncated, info
