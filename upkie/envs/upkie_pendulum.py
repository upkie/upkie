#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron
# Copyright 2023 Inria

## \namespace upkie.envs.upkie_pendulum
## \brief Environment where Upkie behaves like a wheeled inverted pendulum.

import math
from typing import Dict, Optional, Tuple

import gymnasium as gym
import numpy as np

from upkie.config import ROBOT_CONFIG
from upkie.envs.upkie_env import UpkieEnv
from upkie.exceptions import UpkieException
from upkie.logging import logger
from upkie.utils.clamp import clamp_and_warn
from upkie.utils.filters import low_pass_filter


class UpkiePendulum(gym.Wrapper):
    r"""!
    Wrapper to make Upkie act as a wheeled inverted pendulum.

    \anchor upkie_pendulum_description

    When this wrapper is applied, Upkie keeps its legs straight and actions
    only affect wheel velocities. This way, it behaves like a <a
    href="https://scaron.info/robotics/wheeled-inverted-pendulum-model.html">wheeled
    inverted pendulum</a>.

    \note For reinforcement learning with neural-network policies: the
    observation space and action space are not normalized.

    ### Action space

    The action corresponds to the ground velocity resulting from wheel
    velocities. The action vector is simply:

    \f[
    a =\begin{bmatrix} \dot{p}^* \end{bmatrix}
    \f]

    where we denote by \f$\dot{p}^*\f$ the commanded ground velocity in m/s,
    which is internally converted into wheel velocity commands. Note that,
    while this action is not normalized, [-1, 1] m/s is a reasonable range for
    ground velocities.

    ### Observation space

    Vectorized observations have the following structure:

    \f[
    \begin{align*}
    o &= \begin{bmatrix} \theta \\ p \\ \dot{\theta} \\ \dot{p} \end{bmatrix}
    \end{align*}
    \f]

    where we denote by:

    - \f$\theta\f$ the pitch angle of the base with respect to the world
      vertical, in radians. This angle is positive when the robot leans
      forward.
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

    ## \var env
    ## Internal \ref upkie.envs.upkie_env.UpkieEnv environment.
    env: UpkieEnv

    ## \var fall_pitch
    ## Fall detection pitch angle, in radians.
    fall_pitch: float

    ## \var left_wheeled
    ## Set to True (default) if the robot is left wheeled, that is, a positive
    ## turn of the left wheel results in forward motion. Set to False for a
    ## right-wheeled variant.
    left_wheeled: bool

    ## \var observation_space
    ## Observation space.
    observation_space: gym.spaces.Box

    def __init__(
        self,
        env: UpkieEnv,
        fall_pitch: float = 1.0,
        left_wheeled: bool = True,
        max_ground_velocity: float = 1.0,
    ):
        r"""!
        Initialize environment.

        \param env Upkie environment to command servomotors.
        \param fall_pitch Fall detection pitch angle, in radians.
        \param left_wheeled Set to True (default) if the robot is left wheeled,
            that is, a positive turn of the left wheel results in forward
            motion. Set to False for a right-wheeled variant.
        \param max_ground_velocity Maximum commanded ground velocity in m/s.
            The default value of 1 m/s is conservative, don't hesitate to
            increase it once you feel confident in your agent.
        """
        super().__init__(env)
        if env.frequency is None:
            raise UpkieException("This environment needs a loop frequency")

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
            dtype=np.float32,
        )
        action_limit = np.array([max_ground_velocity], dtype=np.float32)

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

        # Instance attributes
        self.__leg_servo_action = env.get_neutral_action()
        self.env = env
        self.fall_pitch = fall_pitch
        self.left_wheeled = left_wheeled

    def __get_env_observation(self, spine_observation: dict) -> np.ndarray:
        r"""!
        Extract environment observation from spine observation dictionary.

        \param spine_observation Spine observation dictionary.
        \return Environment observation vector.
        """
        base_orientation = spine_observation["base_orientation"]
        pitch_base_in_world = base_orientation["pitch"]
        angular_velocity_base_in_base = base_orientation["angular_velocity"]
        ground_position = spine_observation["wheel_odometry"]["position"]
        ground_velocity = spine_observation["wheel_odometry"]["velocity"]

        obs = np.empty(4, dtype=np.float32)
        obs[0] = pitch_base_in_world
        obs[1] = ground_position
        obs[2] = angular_velocity_base_in_base[1]
        obs[3] = ground_velocity
        return obs

    def reset(
        self,
        *,
        seed: Optional[int] = None,
        options: Optional[dict] = None,
    ) -> Tuple[np.ndarray, Dict]:
        r"""!
        Resets the environment and get an initial observation.

        \param seed Number used to initialize the environment’s internal random
            number generator.
        \param options Currently unused.
        \return
            - `observation`: Initial vectorized observation, i.e. an element
              of the environment's `observation_space`.
            - `info`: Dictionary with auxiliary diagnostic information. For
              Upkie this is the full observation dictionary sent by the spine.
        """
        _, info = self.env.reset(seed=seed, options=options)
        spine_observation = info["spine_observation"]
        for joint in self.env.model.upper_leg_joints:
            position = spine_observation["servo"][joint.name]["position"]
            self.__leg_servo_action[joint.name]["position"] = position
        observation = self.__get_env_observation(spine_observation)
        return observation, info

    def __get_leg_servo_action(self) -> Dict[str, Dict[str, float]]:
        r"""!
        Get servo actions for both hip and knee joints.

        \return Servo action dictionary.
        """
        for joint in self.env.model.upper_leg_joints:
            prev_position = self.__leg_servo_action[joint.name]["position"]
            new_position = low_pass_filter(
                prev_output=prev_position,
                new_input=0.0,  # go to neutral configuration
                cutoff_period=1.0,  # in roughly one second
                dt=self.env.dt,
            )
            self.__leg_servo_action[joint.name]["position"] = new_position
        return self.__leg_servo_action

    def __get_wheel_servo_action(
        self, left_wheel_velocity: float
    ) -> Dict[str, Dict[str, float]]:
        r"""!
        Get servo actions for wheel joints.

        \param[in] left_wheel_velocity Left-wheel velocity, in rad/s.
        \return Servo action dictionary.
        """
        right_wheel_velocity = -left_wheel_velocity
        servo_action = {
            "left_wheel": {
                "position": math.nan,
                "velocity": left_wheel_velocity,
            },
            "right_wheel": {
                "position": math.nan,
                "velocity": right_wheel_velocity,
            },
        }
        for joint in self.env.model.wheel_joints:
            servo_action[joint.name]["maximum_torque"] = joint.limit.effort
        return servo_action

    def __get_spine_action(self, action: np.ndarray) -> Dict[str, dict]:
        r"""!
        Convert environment action to a spine action dictionary.

        \param action Environment action.
        \return Spine action dictionary.
        """
        ground_velocity = clamp_and_warn(
            action[0],
            self.action_space.low[0],
            self.action_space.high[0],
            label="ground_velocity",
        )
        wheel_velocity = ground_velocity / ROBOT_CONFIG["wheel_radius"]
        left_wheel_sign = 1.0 if self.left_wheeled else -1.0
        left_wheel_velocity = left_wheel_sign * wheel_velocity
        leg_servo_action = self.__get_leg_servo_action()
        wheel_servo_action = self.__get_wheel_servo_action(left_wheel_velocity)
        return leg_servo_action | wheel_servo_action  # wheel comes second

    def __detect_fall(self, spine_observation: dict) -> bool:
        r"""!
        Detect a fall based on the base-to-world pitch angle.

        \param spine_observation Spine observation dictionary.
        \return True if and only if a fall is detected.

        Spine observations should have a "base_orientation" key. This requires
        the \ref upkie::cpp::observers::BaseOrientation observer in the spine's
        observer pipeline.
        """
        pitch = spine_observation["base_orientation"]["pitch"]
        if abs(pitch) > self.fall_pitch:
            logger.warning(
                "Fall detected (pitch=%.2f rad, fall_pitch=%.2f rad)",
                abs(pitch),
                self.fall_pitch,
            )
            return True
        return False

    def step(
        self,
        action: np.ndarray,
    ) -> Tuple[np.ndarray, float, bool, bool, dict]:
        r"""!
        Run one timestep of the environment's dynamics.

        When the end of the episode is reached, you are responsible for calling
        `reset()` to reset the environment's state.

        \param action Action from the agent.
        \return
            - `observation`: Observation of the environment, i.e. an element
              of its `observation_space`.
            - `reward`: Reward returned after taking the action.
            - `terminated`: Whether the agent reached a terminal state,
              which may be a good or a bad thing. When true, the user needs to
              call `reset()`.
            - `truncated`: Whether the episode is reaching max number of
              steps. This boolean can signal a premature end of the episode,
              i.e. before a terminal state is reached. When true, the user
              needs to call `reset()`.
            - `info`: Dictionary with additional information, reporting in
              particular the full observation dictionary coming from the spine.
        """
        spine_action = self.__get_spine_action(action)
        _, reward, terminated, truncated, info = self.env.step(spine_action)
        spine_observation = info["spine_observation"]
        observation = self.__get_env_observation(spine_observation)
        if self.__detect_fall(spine_observation):
            terminated = True
        return observation, reward, terminated, truncated, info
