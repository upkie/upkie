#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron
# Copyright 2023 Inria

import math
from typing import Dict, Optional, Tuple

import numpy as np
from gymnasium import spaces
from numpy.typing import NDArray

from upkie.utils.exceptions import UpkieException
from upkie.utils.filters import low_pass_filter
from upkie.utils.robot_state import RobotState

from .upkie_base_env import UpkieBaseEnv


class UpkieGroundVelocity(UpkieBaseEnv):
    """!
    Environment where Upkie is used as a wheeled inverted pendulum.

    The environment id is ``UpkieGroundVelocity-v3``. Model assumptions are
    summarized in the <a
    href="https://scaron.info/robotics/wheeled-inverted-pendulum-model.html">following
    note</a>.

    @note For reinforcement learning with neural networks: the observation
    space and action space are not normalized.

    ### Action space

    The action corresponds to the ground velocity resulting from wheel
    velocities. The action vector is simply:

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

    As with all Upkie environments, full observations from the spine (detailed
    in @ref observations) are also available in the ``info`` dictionary
    returned by the reset and step functions.
    """

    class RewardWeights:
        """Weights of the position and velocity terms in rewards."""

        ## @var position
        ## Weight of the position term.
        position: float

        ## @var velocity
        ## Weight of the velocity term.
        velocity: float

        def __init__(self, position: float = 1.0, velocity: float = 1.0):
            """!
            Initialize reward weights.

            @param position Weight of the position term.
            @param velocity Weight of the velocity term.
            """
            self.position = position
            self.velocity = velocity

    ## @var action_space
    ## Action space.
    action_space: spaces.box.Box

    ## @var leg_return_period
    ## Time constant for the legs (hips and knees) to revert to their neutral
    ## configuration.
    leg_return_period: float

    ## @var observation_space
    ## Observation space.
    observation_space: spaces.box.Box

    ## @var reward_weights
    ## Weights of the position and velocity terms in rewards.
    reward_weights: "UpkieGroundVelocity.RewardWeights"

    ## @var wheel_radius
    ## Wheel radius in [m].
    wheel_radius: float

    def __init__(
        self,
        fall_pitch: float = 1.0,
        frequency: float = 200.0,
        frequency_checks: bool = True,
        init_state: Optional[RobotState] = None,
        leg_return_period: float = 1.0,
        max_ground_velocity: float = 1.0,
        regulate_frequency: bool = True,
        reward_weights: Optional[RewardWeights] = None,
        shm_name: str = "/vulp",
        spine_config: Optional[dict] = None,
        wheel_radius: float = 0.06,
    ):
        """!
        Initialize environment.

        @param fall_pitch Fall detection pitch angle, in radians.
        @param frequency Regulated frequency of the control loop, in Hz.
        @param frequency_checks If `regulate_frequency` is set and this
            parameter is true (default), a warning is issued every time the
            control loop runs slower than the desired `frequency`. Set this
            parameter to false to disable these warnings.
        @param init_state Initial state of the robot, only used in simulation.
        @param leg_return_period Time constant for the legs (hips and knees) to
            revert to their neutral configuration.
        @param max_ground_velocity Maximum commanded ground velocity in m/s.
        @param regulate_frequency Enables loop frequency regulation.
        @param reward_weights Coefficients before each reward term.
        @param shm_name Name of shared-memory file.
        @param spine_config Additional spine configuration overriding the
            defaults from ``//config:spine.yaml``. The combined configuration
            dictionary is sent to the spine at every :func:`reset`.
        @param wheel_radius Wheel radius in [m].
        """
        super().__init__(
            fall_pitch=fall_pitch,
            frequency=frequency,
            frequency_checks=frequency_checks,
            init_state=init_state,
            regulate_frequency=regulate_frequency,
            shm_name=shm_name,
            spine_config=spine_config,
        )

        if self.dt is None:
            raise UpkieException("This environment needs a loop frequency")

        reward_weights: UpkieGroundVelocity.RewardWeights = (
            reward_weights
            if reward_weights is not None
            else UpkieGroundVelocity.RewardWeights()
        )

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

        self.__leg_servo_action = {
            joint: {
                "position": None,
                "velocity": 0.0,
                "maximum_torque": 10.0,  # qdd100 actuators
            }
            for joint in self.model.leg_joints
        }

        self.leg_return_period = leg_return_period
        self.reward_weights = reward_weights
        self.version = UpkieGroundVelocity.version  # for Doxygen...
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
        return super().reset(seed=seed)

    def parse_first_observation(self, spine_observation: dict) -> None:
        """!
        Parse first observation after the spine interface is initialized.

        @param spine_observation First observation.
        """
        for joint in self.LEG_JOINTS:
            position = spine_observation["servo"][joint]["position"]
            self.__leg_servo_action[joint]["position"] = position

    def get_env_observation(self, spine_observation: dict) -> NDArray[float]:
        """!
        Extract environment observation from spine observation dictionary.

        @param spine_observation Spine observation dictionary.
        @returns Environment observation vector.
        """
        base_orientation = spine_observation["base_orientation"]
        pitch_base_in_world = base_orientation["pitch"]
        angular_velocity_base_in_base = base_orientation["angular_velocity"]
        ground_position = spine_observation["wheel_odometry"]["position"]
        ground_velocity = spine_observation["wheel_odometry"]["velocity"]

        obs = np.empty(4, dtype=float)
        obs[0] = pitch_base_in_world
        obs[1] = ground_position
        obs[2] = angular_velocity_base_in_base[1]
        obs[3] = ground_velocity
        return obs

    def get_leg_servo_action(self) -> Dict[str, Dict[str, float]]:
        """!
        Get servo actions for both hip and knee joints.

        @returns Servo action dictionary.
        """
        for joint in self.LEG_JOINTS:
            prev_position = self.__leg_servo_action[joint]["position"]
            new_position = low_pass_filter(
                prev_output=prev_position,
                cutoff_period=self.leg_return_period,
                new_input=0.0,
                dt=self.dt,
            )
            self.__leg_servo_action[joint]["position"] = new_position
        return self.__leg_servo_action.copy()

    def get_spine_action(self, action: NDArray[float]) -> dict:
        """!
        Convert environment action to a spine action dictionary.

        @param action Environment action.
        @returns Spine action dictionary.
        """
        ground_velocity = action[0]
        wheel_velocity = ground_velocity / self.wheel_radius
        servo_dict = self.get_leg_servo_action()
        servo_dict.update(
            {
                "left_wheel": {
                    "position": math.nan,
                    "velocity": +wheel_velocity,
                    "maximum_torque": 1.0,  # mj5208 actuator
                },
                "right_wheel": {
                    "position": math.nan,
                    "velocity": -wheel_velocity,
                    "maximum_torque": 1.0,  # mj5208 actuator
                },
            }
        )
        spine_action = {"servo": servo_dict}
        return spine_action

    def get_reward(
        self,
        observation: NDArray[float],
        action: NDArray[float],
    ) -> float:
        """!
        Get reward from observation and action.

        @param observation Environment observation vector.
        @param action Environment action vector.
        @returns Reward.
        """
        pitch = observation[0]
        ground_position = observation[1]
        angular_velocity = observation[2]
        ground_velocity = observation[3]

        tip_height = 0.58  # [m]
        tip_position = ground_position + tip_height * np.sin(pitch)
        tip_velocity = (
            ground_velocity + tip_height * angular_velocity * np.cos(pitch)
        )

        std_position = 0.05  # [m]
        position_reward = np.exp(-((tip_position / std_position) ** 2))
        velocity_penalty = -abs(tip_velocity)

        return (
            self.reward_weights.position * position_reward
            + self.reward_weights.velocity * velocity_penalty
        )
