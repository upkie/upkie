#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
#
## \namespace upkie.envs.upkie_gyropod
## \brief Environment where Upkie behaves like a wheeled inverted pendulum
## with yaw control.

import math
from typing import Dict, Optional, Tuple

import gymnasium as gym
import numpy as np

from upkie.envs.upkie_servos import UpkieServos
from upkie.exceptions import UpkieException
from upkie.logging import logger
from upkie.utils.clamp import clamp_and_warn
from upkie.utils.filters import low_pass_filter


class UpkieGyropod(gym.Wrapper):
    r"""!
    Wrapper to make Upkie act as a wheeled inverted pendulum with yaw control.

    \anchor upkie_gyropod_description

    When this wrapper is applied, Upkie keeps its legs straight and actions
    affect wheel velocity commands. Overall, the robot behaves like a <a
    href="https://scaron.info/robotics/wheeled-inverted-pendulum-model.html">wheeled
    inverted pendulum</a> in the sagittal plane, and turns by <a
    href="https://en.wikipedia.org/wiki/Differential_wheeled_robot">differential
    drive</a>.

    \note Note if you are doing reinforcement learning with neural-network
    policies that the observation space and action space are not normalized.

    ### Action space

    The action corresponds to the ground sagittal velocity and yaw velocity:

    \f[
    a = \begin{bmatrix} \dot{p}^* \\ \dot{\psi}^* \end{bmatrix}
    \f]

    where \f$\dot{p}^*\f$ is the commanded ground velocity in m/s and
    \f$\dot{\psi}^*\f$ is the commanded yaw velocity in rad/s. Both are
    internally converted into wheel velocity commands.

    ### Observation space

    Vectorized observations have the following structure:

    \f[
    o = \begin{bmatrix}
        p \\ \theta \\ \psi \\
        \dot{p} \\ \dot{\theta} \\ \dot{\psi}
    \end{bmatrix}
    \f]

    where we denote by:

    - \f$p \in \mathbb{R}\f$ the position of the average wheel contact point,
      in meters. This is only a 1D position: it is an x-coordinate as long as
      the robot keeps the same sagittal plane, once it turns \f$p\f$ should be
      considered a curvilinear abscissa.
    - \f$\theta\f$ the pitch angle of the base with respect to the world
      vertical, in radians. This angle is positive when the robot leans
      forward.
    - \f$\psi\f$ the yaw angle, in radians, integrated from the commanded
      yaw velocity. It resets to zero at each episode.
    - \f$\dot{p}\f$ the velocity of the average wheel contact point, in meters
      per seconds.
    - \f$\dot{\theta}\f$ the body angular velocity of the base frame along its
      lateral axis, in radians per seconds.
    - \f$\dot{\psi}\f$ the commanded yaw velocity, in radians per second.

    \note The pitch velocity \f$\dot{\theta}\f$ is the body-frame angular
    velocity from the IMU, not the exact time derivative of the pitch Euler
    angle. Similarly, the yaw velocity \f$\dot{\psi}\f$ is the commanded value.
    A future improvement could compute both from the full orientation and
    angular velocity.

    As with all Upkie environments, full observations from the spine (detailed
    in \ref observations) are also available in the `info` dictionary
    returned by the reset and step functions.
    """

    ## \var action_space
    ## Action space.
    action_space: gym.spaces.Box

    ## \var fall_pitch
    ## Fall detection pitch angle, in radians.
    fall_pitch: float

    ## \var leg_gain_scale
    ## Gain scale applied to upper leg kp_scale and kd_scale. Can be updated
    ## per-step by the agent (e.g. to stiffen legs during turns).
    leg_gain_scale: float

    ## \var observation_space
    ## Observation space.
    observation_space: gym.spaces.Box

    def __init__(
        self,
        servos_env: UpkieServos,
        fall_pitch: float = 1.0,
        leg_gain_scale: float = 1.0,
        max_ground_velocity: float = 3.0,
        max_yaw_velocity: float = 1.0,
    ):
        r"""!
        Initialize environment.

        \param servos_env Upkie environment to command servomotors.
        \param fall_pitch Fall detection pitch angle, in radians.
        \param leg_gain_scale Gain scale applied to upper leg kp_scale and
            kd_scale servo parameters.
        \param max_ground_velocity Maximum commanded ground velocity in m/s.
        \param max_yaw_velocity Maximum commanded yaw velocity in rad/s.
        """
        super().__init__(servos_env)
        if self.env.frequency is None:
            raise UpkieException("This environment needs a loop frequency")

        MAX_GROUND_POSITION: float = float("inf")
        MAX_BASE_PITCH: float = np.pi
        MAX_YAW_ANGLE: float = float("inf")
        MAX_GROUND_VELOCITY: float = max_ground_velocity
        MAX_PITCH_VELOCITY: float = 1000.0  # rad/s
        MAX_YAW_VELOCITY: float = max_yaw_velocity
        observation_limit = np.array(
            [
                MAX_GROUND_POSITION,
                MAX_BASE_PITCH,
                MAX_YAW_ANGLE,
                MAX_GROUND_VELOCITY,
                MAX_PITCH_VELOCITY,
                MAX_YAW_VELOCITY,
            ],
            dtype=np.float32,
        )
        action_limit = np.array(
            [max_ground_velocity, max_yaw_velocity],
            dtype=np.float32,
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

        # Instance attributes
        self.__leg_gain_scale = leg_gain_scale
        self.__leg_servo_action = self.env.get_neutral_action()
        self.__yaw_angle = 0.0  # rad
        self.__yaw_velocity = 0.0  # rad/s
        self.fall_pitch = fall_pitch

    @property
    def leg_gain_scale(self) -> float:
        r"""!
        Get the current kp/kd scale upplied to hip and knee joints.
        """
        return self.__leg_gain_scale

    def set_leg_gain_scale(self, leg_gain_scale: float) -> None:
        r"""!
        Set a new kp/kd scale upplied to hip and knee joints.

        \param leg_gain_scale New kp/kd scale.
        """
        self.__leg_gain_scale = leg_gain_scale

    def __get_env_observation(
        self, spine_observation: dict, yaw_angle: float, yaw_velocity: float
    ) -> np.ndarray:
        r"""!
        Extract environment observation from spine observation dictionary.

        \param spine_observation Spine observation dictionary.
        \param yaw_angle Current yaw angle in radians, added as a parameter
            here to keep track of it being an internal environment state for
            now.
        \param yaw_velocity Current yaw velocity in rad/s, added as a parameter
            here to keep track of it being an internal environment state for
            now.
        \return Environment observation vector.
        """
        base_orientation = spine_observation["base_orientation"]
        pitch = base_orientation["pitch"]
        angular_velocity_base_in_base = base_orientation["angular_velocity"]
        ground_position = spine_observation["wheel_odometry"]["position"]
        ground_velocity = spine_observation["wheel_odometry"]["velocity"]

        obs = np.empty(6, dtype=np.float32)
        obs[0] = ground_position
        obs[1] = pitch
        obs[2] = yaw_angle
        obs[3] = ground_velocity
        obs[4] = angular_velocity_base_in_base[1]
        obs[5] = yaw_velocity
        return obs

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
        spine_observation = info["spine_observation"]
        for joint in self.env.model.upper_leg_joints:
            position = spine_observation["servo"][joint.name]["position"]
            self.__leg_servo_action[joint.name]["position"] = position
        self.__yaw_angle = 0.0  # rad
        self.__yaw_velocity = 0.0  # rad/s
        observation = self.__get_env_observation(
            spine_observation, self.__yaw_angle, self.__yaw_velocity
        )
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
            self.__leg_servo_action[joint.name]["kp_scale"] = (
                self.__leg_gain_scale
            )
            self.__leg_servo_action[joint.name]["kd_scale"] = (
                self.__leg_gain_scale
            )
        return self.__leg_servo_action

    def __get_wheel_servo_action(
        self, left_wheel_velocity: float, right_wheel_velocity: float
    ) -> Dict[str, Dict[str, float]]:
        r"""!
        Get servo actions for wheel joints.

        \param left_wheel_velocity Left-wheel velocity, in rad/s.
        \param right_wheel_velocity Right-wheel velocity, in rad/s.
        \return Servo action dictionary.
        """
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

        \param action Environment action: [ground_velocity, yaw_velocity].
        \return Spine action dictionary.
        """
        ground_velocity = clamp_and_warn(
            action[0],
            self.action_space.low[0],
            self.action_space.high[0],
            label="ground_velocity",
        )
        yaw_velocity = clamp_and_warn(
            action[1],
            self.action_space.low[1],
            self.action_space.high[1],
            label="yaw_velocity",
        )

        model = self.env.model

        # Sagittal translation
        wheel_velocity = ground_velocity / model.wheel_radius
        left_sign = 1.0 if model.left_wheeled else -1.0
        left_wheel_velocity = left_sign * wheel_velocity
        right_wheel_velocity = -left_sign * wheel_velocity

        # Yaw rotation
        contact_radius = 0.5 * model.wheel_base
        yaw_to_wheel = left_sign * contact_radius / model.wheel_radius
        left_wheel_velocity += yaw_to_wheel * yaw_velocity
        right_wheel_velocity += yaw_to_wheel * yaw_velocity

        leg_servo_action = self.__get_leg_servo_action()
        wheel_servo_action = self.__get_wheel_servo_action(
            left_wheel_velocity, right_wheel_velocity
        )
        return leg_servo_action | wheel_servo_action

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
                "Fall detected (pitch = %.2f rad >= %.2f rad)",
                pitch,
                (+1.0 if pitch > 0 else -1.0) * self.fall_pitch,
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

        \param action Action from the agent: [ground_velocity, yaw_velocity].
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

        yaw_velocity = action[1]
        self.__yaw_angle += yaw_velocity * self.env.dt
        self.__yaw_velocity = yaw_velocity
        observation = self.__get_env_observation(
            spine_observation, self.__yaw_angle, self.__yaw_velocity
        )

        if self.__detect_fall(spine_observation):
            terminated = True
        return observation, reward, terminated, truncated, info
