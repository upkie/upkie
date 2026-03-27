#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Upkie Team

from typing import Dict, Optional, Tuple

import gymnasium as gym
import numpy as np

from upkie.controllers.mpc_balancer import MPCBalancer
from upkie.envs.upkie_pendulum import UpkiePendulum
from upkie.utils.clamp import clamp_and_warn


class UpkieNavigation(gym.Wrapper):
    r"""!
    Wrapper to make Upkie act as a differential drive robot in SE(2).

    This wrapper allows Upkie to navigate in the 2D plane by keeping its legs
    straight and commanding wheel velocities for differential drive motion. The
    robot's pose \f[(x, y, \theta)\f] is estimated by wheel odometry.

    ### Action space

    The action corresponds to desired linear and angular velocities:

    \f[
    a = \begin{bmatrix} v \\ \omega \end{bmatrix}
    \f]

    where:
    - \f$v\f$ is the linear velocity in m/s (forward/backward)
    - \f$\omega\f$ is the angular velocity in rad/s (yaw rotation)

    ### Observation space

    The observation is the robot's pose in SE(2):

    \f[
    o = \begin{bmatrix} x \\ y \\ \theta \end{bmatrix}
    \f]

    where:
    - \f$x\f$ is the position along the x-axis in meters
    - \f$y\f$ is the position along the y-axis in meters
    - \f$\theta\f$ is the yaw angle in radians
    """

    action_space: gym.spaces.Box
    env: UpkiePendulum
    mpc_balancer: MPCBalancer
    observation_space: gym.spaces.Box

    def __init__(
        self,
        env: UpkiePendulum,
        max_linear_velocity: float = 1.0,
        max_angular_velocity: float = 2.0,
    ):
        r"""!
        Initialize navigation environment.

        \param env UpkiePendulum environment to wrap.
        \param max_linear_velocity Maximum linear velocity in m/s.
        \param max_angular_velocity Maximum angular velocity in rad/s.
        """
        super().__init__(env)

        # Observation limits for SE(2) pose
        MAX_POSITION: float = float("inf")
        MAX_ANGLE: float = np.pi
        observation_limit = np.array(
            [MAX_POSITION, MAX_POSITION, MAX_ANGLE],
            dtype=float,
        )

        # Action limits for [linear_velocity, angular_velocity]
        action_limit = np.array(
            [max_linear_velocity, max_angular_velocity], dtype=np.float32
        )

        # gymnasium.Env: observation_space
        self.observation_space = gym.spaces.Box(
            -observation_limit,
            +observation_limit,
            shape=observation_limit.shape,
            dtype=observation_limit.dtype,
        )

        # gymnasium.Env: action_space
        self.action_space = gym.spaces.Box(
            -action_limit,
            +action_limit,
            shape=action_limit.shape,
            dtype=action_limit.dtype,
        )

        # MPC balancer
        mpc_balancer = MPCBalancer(
            leg_length=0.58,  # Default leg length
            fall_pitch=1.0,  # Use default fall pitch
            max_ground_accel=1.0,
            max_ground_velocity=max_linear_velocity,
        )

        # Instance attributes
        self.__last_spine_observation = {}
        self.env = env
        self.mpc_balancer = mpc_balancer
        self.last_ground_position = 0.0
        self.pose = np.zeros(3)  # [x, y, theta]

    def __get_env_observation(self, spine_observation: dict) -> np.ndarray:
        r"""!
        Extract environment observation from spine observation dictionary.

        \param spine_observation Spine observation dictionary.
        \return Environment observation vector (SE(2) pose).
        """
        cur_ground_position = spine_observation["wheel_odometry"]["position"]
        displacement = cur_ground_position - self.last_ground_position
        self.last_ground_position = cur_ground_position

        # Update pose by integrating displacement
        self.pose[0] += displacement * np.cos(self.pose[2])  # x
        self.pose[1] += displacement * np.sin(self.pose[2])  # y
        # theta is updated in step() based on target yaw velocity

        return self.pose.copy()

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
            - `observation`: Initial SE(2) pose observation.
            - `info`: Dictionary with auxiliary diagnostic information.
        """
        _, info = self.env.reset(seed=seed, options=options)
        spine_observation = info["spine_observation"]
        self.__last_spine_observation = spine_observation

        # Reset pose tracking
        self.pose = np.zeros(3)
        self.last_ground_position = spine_observation["wheel_odometry"][
            "position"
        ]

        observation = self.__get_env_observation(spine_observation)
        return observation, info

    def step(
        self,
        action: np.ndarray,
    ) -> Tuple[np.ndarray, float, bool, bool, dict]:
        r"""!
        Run one timestep of the environment's dynamics.

        \param action Action from the agent [linear, angular velocities].
        \return
            - `observation`: SE(2) pose observation.
            - `reward`: Reward returned after taking the action.
            - `terminated`: Whether the agent reached a terminal state.
            - `truncated`: Whether the episode is reaching max number of steps.
            - `info`: Dictionary with additional information.
        """
        # Clamp actions
        target_ground_velocity = clamp_and_warn(
            action[0],
            self.action_space.low[0],
            self.action_space.high[0],
            label="linear_velocity",
        )
        target_yaw_velocity = clamp_and_warn(
            action[1],
            self.action_space.low[1],
            self.action_space.high[1],
            label="angular_velocity",
        )

        # Update theta estimate based on target angular velocity
        dt = self.env.unwrapped.dt
        self.pose[2] += target_yaw_velocity * dt
        # Normalize angle to [-pi, pi]
        self.pose[2] = np.arctan2(np.sin(self.pose[2]), np.cos(self.pose[2]))

        # Use MPC balancer to compute balanced ground velocity
        ground_velocity = self.mpc_balancer.compute_ground_velocity(
            target_ground_velocity, self.__last_spine_observation, dt
        )

        # Call the UpkiePendulum step function
        _, reward, terminated, truncated, info = self.env.step(
            action=np.array([ground_velocity])
        )

        # Get our navigation observation
        spine_observation = info["spine_observation"]
        observation = self.__get_env_observation(spine_observation)
        self.__last_spine_observation = spine_observation

        return observation, reward, terminated, truncated, info
