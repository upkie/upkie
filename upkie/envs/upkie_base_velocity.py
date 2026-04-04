#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
#
## \namespace upkie.envs.upkie_base_velocity
## \brief Environment where Upkie is balanced and controlled via base velocity.

from __future__ import annotations

import math
from typing import TYPE_CHECKING, Dict, Optional, Tuple

import gymnasium as gym
import numpy as np

from upkie.envs.upkie_gyropod import UpkieGyropod

if TYPE_CHECKING:
    from upkie.controllers.mpc_balancer import MPCBalancer
    from upkie.envs.upkie_servos import UpkieServos


class UpkieBaseVelocity(gym.Wrapper):
    r"""!
    Wrapper where the Upkie balances itself and the action is directly a base
    velocity.

    \anchor upkie_base_velocity_description

    This wrapper embeds an \ref upkie.controllers.mpc_balancer.MPCBalancer
    "MPCBalancer" to handle sagittal balance internally. The user commands
    the robot's base velocity in se(2) and observes its SE(2) pose in the
    horizontal plane.

    ### Action space

    The action is an element of \f$\mathfrak{se}(2)\f$:

    \f[
    a = \begin{bmatrix} v \\ \dot{\psi}^* \end{bmatrix}
    \f]

    where \f$v\f$ is the target linear velocity in m/s and
    \f$\dot{\psi}^*\f$ is the commanded yaw velocity in rad/s. The linear
    velocity is tracked by the internal MPC balancer, which computes the
    ground velocity needed to maintain balance while following the target.

    ### Observation space

    Observations are the robot's SE(2) pose in the horizontal plane:

    \f[
    o = \begin{bmatrix} x \\ y \\ \psi \end{bmatrix}
    \f]

    where \f$x\f$ and \f$y\f$ are positions in meters, dead-reckoned from
    the commanded linear velocity and current yaw, and \f$\psi\f$ is the
    yaw angle in radians (accumulated freely, not wrapped).

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
        max_yaw_velocity: float = 1.0,
        leg_length: float = 0.58,
        max_ground_accel: float = 10.0,
    ):
        r"""!
        Initialize environment.

        \param servos Upkie environment to command servomotors.
        \param fall_pitch Fall detection pitch angle, in radians.
        \param max_ground_velocity Maximum commanded ground velocity in m/s.
        \param max_yaw_velocity Maximum commanded yaw velocity in rad/s.
        \param leg_length Leg length in meters, forwarded to the internal
            MPC balancer.
        \param max_ground_accel Maximum ground acceleration in m/s², forwarded
            to the internal MPC balancer.
        """
        gyropod_env = UpkieGyropod(
            servos_env,
            fall_pitch=fall_pitch,
            max_ground_velocity=max_ground_velocity,
            max_yaw_velocity=max_yaw_velocity,
        )
        super().__init__(gyropod_env)

        observation_limit = np.full(3, float("inf"), dtype=np.float32)
        self.observation_space = gym.spaces.Box(
            -observation_limit,
            +observation_limit,
            shape=observation_limit.shape,
            dtype=observation_limit.dtype,
        )

        action_limit = np.array(
            [max_ground_velocity, max_yaw_velocity],
            dtype=np.float32,
        )
        self.action_space = gym.spaces.Box(
            -action_limit,
            +action_limit,
            shape=action_limit.shape,
            dtype=action_limit.dtype,
        )

        from upkie.controllers.mpc_balancer import MPCBalancer

        self.__mpc_balancer = MPCBalancer(
            fall_pitch=fall_pitch,
            leg_length=leg_length,
            max_ground_accel=max_ground_accel,
            max_ground_velocity=max_ground_velocity,
        )
        self.__spine_observation: dict = {}
        self.__x: float = 0.0
        self.__y: float = 0.0

    @property
    def mpc_balancer(self) -> MPCBalancer:
        r"""!
        Get the internal MPC balancer.
        """
        return self.__mpc_balancer

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
            - `info`: Dictionary with auxiliary diagnostic information. For
              Upkie this is the full observation dictionary sent by the spine.
        """
        _, info = self.env.reset(seed=seed, options=options)
        self.__spine_observation = info["spine_observation"]
        self.__mpc_balancer.reset()
        self.__x = 0.0
        self.__y = 0.0
        observation = np.zeros(3, dtype=np.float32)
        return observation, info

    def step(
        self,
        action: np.ndarray,
    ) -> Tuple[np.ndarray, float, bool, bool, dict]:
        r"""!
        Run one timestep of the environment's dynamics.

        \param action Action from the agent: [linear_velocity, yaw_velocity].
        \return
            - `observation`: Observation of the environment, i.e. an element
              of its `observation_space`.
            - `reward`: Reward returned after taking the action.
            - `terminated`: Whether the agent reached a terminal state.
            - `truncated`: Whether the episode is reaching max number of steps.
            - `info`: Dictionary with additional information, reporting in
              particular the full observation dictionary coming from the spine.
        """
        linear_velocity = float(action[0])
        yaw_velocity = float(action[1])

        dt = self.env.unwrapped.dt
        ground_velocity = self.__mpc_balancer.step(
            linear_velocity, self.__spine_observation, dt
        )

        gyropod_action = np.array(
            [ground_velocity, yaw_velocity], dtype=np.float32
        )
        gyropod_obs, reward, terminated, truncated, info = self.env.step(
            gyropod_action
        )
        self.__spine_observation = info["spine_observation"]

        yaw = float(gyropod_obs[2])
        self.__x += linear_velocity * math.cos(yaw) * dt
        self.__y += linear_velocity * math.sin(yaw) * dt

        observation = np.array([self.__x, self.__y, yaw], dtype=np.float32)
        return observation, reward, terminated, truncated, info
