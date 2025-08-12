#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

## \namespace upkie.envs.upkie_servos
## \brief Upkie environment where actions command servomotors directly.

from typing import Optional, Tuple

import gymnasium as gym
import numpy as np

from upkie.envs.backends import Backend
from upkie.exceptions import UpkieRuntimeError
from upkie.utils.clamp import clamp_and_warn
from upkie.utils.robot_state import RobotState

from .upkie_env import UpkieEnv


class UpkieServos(UpkieEnv):
    r"""!
    Upkie environment where actions command servomotors directly.

    \anchor upkie_servos_description

    Actions and observations correspond to the moteus servo API.

    ### Action space

    The action space is a dictionary with one key for each servo:

    - `left_hip`: left hip joint (qdd100)
    - `left_knee`: left knee joint (qdd100)
    - `left_wheel`: left wheel joint (mj5208)
    - `right_hip`: right hip joint (qdd100)
    - `right_knee`: right knee joint (qdd100)
    - `right_wheel`: right wheel joint (mj5208)

    The value for each servo dictionary is itself a dictionary with the
    following keys:

    - `position`: commanded joint angle \f$\theta^*\f$ in [rad] (NaN to
       disable) (required).
    - `velocity`: commanded joint velocity \f$\dot{\theta}^*\f$ in [rad] /
       [s] (required).
    - `feedforward_torque`: feedforward joint torque \f$\tau_{\mathit{ff}}\f$
       in [N m].
    - `kp_scale`: scaling factor \f$k_{p}^{\mathit{scale}}\f$ applied to the
       position feedback gain, between zero and one.
    - `kd_scale`: scaling factor \f$k_{d}^{\mathit{scale}}\f$ applied to the
       velocity feedback gain, between zero and one.
    - `maximum_torque`: maximum joint torque \f$\tau_{\mathit{max}}\f$
       (feedforward + feedback) enforced during the whole actuation step, in
       [N m].

    The resulting torque applied by the servo is then:

    \f[
    \begin{align*}
    \tau & = \underset{
            [-\tau_{\mathit{max}}, +\tau_{\mathit{max}}]}{
            \mathrm{clamp}
        }
        \left(
            \tau_{\mathit{ff}} +
            k_{p} k_{p}^{\mathit{scale}} (\theta^* - \theta) +
            k_{d} k_{d}^{\mathit{scale}} (\dot{\theta}^* - \dot{\theta}))
        \right)
    \end{align*}
    \f]

    Position and velocity gains \f$k_{p}\f$ and \f$k_{d}\f$ are configured in
    each moteus controller directly and don't change during execution. We can
    rather modulate the overall feedback gains via the normalized parameters
    \f$k_{p}^{\mathit{scale}} \in [0, 1]\f$ and \f$k_{d}^{\mathit{scale}} \in
    [0, 1]\f$. Note that the servo regulates the torque above at its own
    frequency, which is higher (typically 40 kHz) than the agent and the spine
    frequencies. See the [moteus
    reference](https://github.com/mjbots/moteus/blob/13c171c697ce6f60a73c9385e6fe951957313d1d/docs/reference.md#theory-of-operation)
    for more details.

    ### Observation space

    The observation space is a dictionary with one key for each servo. The
    value for each key is a dictionary with keys:

    - `position`: Joint angle in [rad].
    - `velocity`: Joint velocity in [rad] / [s].
    - `torque`: Joint torque in [N m].
    - `temperature`: Servo temperature in degree Celsius.
    - `voltage`: Power bus voltage of the servo, in [V].

    Full observations from the backend (detailed in \ref observations) are also
    available in the `info` dictionary returned by the reset and step
    functions.
    """

    ACTION_KEYS: Tuple[str, str, str, str, str, str] = (
        "position",
        "velocity",
        "feedforward_torque",
        "kp_scale",
        "kd_scale",
        "maximum_torque",
    )

    ## \var action_space
    ## Action space of the environment.
    action_space: gym.Space

    ## \var observation_space
    ## Observation space of the environment.
    observation_space: gym.Space

    def __init__(
        self,
        backend: Backend,
        frequency: Optional[float] = 200.0,
        frequency_checks: bool = True,
        init_state: Optional[RobotState] = None,
        regulate_frequency: bool = True,
        max_gain_scale: float = 5.0,
    ) -> None:
        r"""!
        Initialize servos environment.

        \param backend Backend for interfacing with a simulator or a spine.
        \param frequency Regulated frequency of the control loop, in Hz. Can be
            prescribed even when `regulate_frequency` is unset, in which case
            `self.dt` will be defined but the loop frequency will not be
            regulated.
        \param frequency_checks If `regulate_frequency` is set and this
            parameter is `True`, a warning will be issued every time the
            control loop runs slower than the desired `frequency`.
        \param init_state Initial state of the robot, only used in simulation.
        \param regulate_frequency If set (default), the environment will
            regulate the control loop frequency to the value prescribed in
            `frequency`.
        \param max_gain_scale Maximum value for kp or kd gain scales.
        """
        if not (0.0 < max_gain_scale < 10.0):
            raise UpkieRuntimeError("Invalid value {max_gain_scale =}")

        # Initialize base class but override action/observation spaces
        super().__init__(
            backend=backend,
            frequency=frequency,
            frequency_checks=frequency_checks,
            init_state=init_state,
            regulate_frequency=regulate_frequency,
        )

        # Initialize action and observation spaces
        (
            action_space,
            observation_space,
            neutral_action,
            max_action,
            min_action,
        ) = self.__create_servo_spaces(max_gain_scale)

        # Override with servo-specific spaces
        self.action_space = action_space
        self.observation_space = observation_space
        self._max_action = max_action
        self._min_action = min_action
        self._neutral_action = neutral_action

    def __create_servo_spaces(self, max_gain_scale: float):
        action_space = {}
        neutral_action = {}
        max_action = {}
        min_action = {}
        servo_space = {}

        for joint in self.model.joints:
            action_space[joint.name] = gym.spaces.Dict(
                {
                    "position": gym.spaces.Box(
                        low=joint.limit.lower,
                        high=joint.limit.upper,
                        shape=(1,),
                        dtype=float,
                    ),
                    "velocity": gym.spaces.Box(
                        low=-joint.limit.velocity,
                        high=+joint.limit.velocity,
                        shape=(1,),
                        dtype=float,
                    ),
                    "feedforward_torque": gym.spaces.Box(
                        low=-joint.limit.effort,
                        high=+joint.limit.effort,
                        shape=(1,),
                        dtype=float,
                    ),
                    "kp_scale": gym.spaces.Box(
                        low=0.0,
                        high=max_gain_scale,
                        shape=(1,),
                        dtype=float,
                    ),
                    "kd_scale": gym.spaces.Box(
                        low=0.0,
                        high=max_gain_scale,
                        shape=(1,),
                        dtype=float,
                    ),
                    "maximum_torque": gym.spaces.Box(
                        low=0.0,
                        high=joint.limit.effort,
                        shape=(1,),
                        dtype=float,
                    ),
                }
            )
            servo_space[joint.name] = gym.spaces.Dict(
                {
                    "position": gym.spaces.Box(
                        low=joint.limit.lower,
                        high=joint.limit.upper,
                        shape=(1,),
                        dtype=float,
                    ),
                    "velocity": gym.spaces.Box(
                        low=-joint.limit.velocity,
                        high=+joint.limit.velocity,
                        shape=(1,),
                        dtype=float,
                    ),
                    "torque": gym.spaces.Box(
                        low=-joint.limit.effort,
                        high=+joint.limit.effort,
                        shape=(1,),
                        dtype=float,
                    ),
                    "temperature": gym.spaces.Box(
                        low=0.0,
                        high=100.0,
                        shape=(1,),
                        dtype=float,
                    ),
                    "voltage": gym.spaces.Box(
                        low=10.0,  # moteus min 10 V
                        high=44.0,  # moteus max 44 V
                        shape=(1,),
                        dtype=float,
                    ),
                }
            )
            neutral_action[joint.name] = {
                "position": np.nan,
                "velocity": 0.0,
                "feedforward_torque": 0.0,
                "kp_scale": 1.0,
                "kd_scale": 1.0,
                "maximum_torque": joint.limit.effort,
            }
            max_action[joint.name] = {
                "position": joint.limit.upper,
                "velocity": joint.limit.velocity,
                "feedforward_torque": joint.limit.effort,
                "kp_scale": max_gain_scale,
                "kd_scale": max_gain_scale,
                "maximum_torque": joint.limit.effort,
            }
            min_action[joint.name] = {
                "position": joint.limit.lower,
                "velocity": -joint.limit.velocity,
                "feedforward_torque": -joint.limit.effort,
                "kp_scale": 0.0,
                "kd_scale": 0.0,
                "maximum_torque": 0.0,
            }

        return (
            gym.spaces.Dict(action_space),
            gym.spaces.Dict(servo_space),
            neutral_action,
            max_action,
            min_action,
        )

    def get_env_observation(self, spine_observation: dict) -> dict:
        r"""!
        Extract environment observation from spine observation dictionary.

        \param spine_observation Full observation dictionary from the spine.
        \return Environment observation.
        """
        # If creating a new object turns out to be too slow we can switch to
        # updating in-place.
        return {
            joint.name: {
                key: np.array(
                    [spine_observation["servo"][joint.name][key]],
                    dtype=float,
                )
                for key in self.observation_space[joint.name]
            }
            for joint in self.model.joints
        }

    def get_neutral_action(self) -> dict:
        r"""!
        Get the neutral action where servos don't move.

        \return Neutral action where servos don't move.
        """
        return self._neutral_action.copy()

    def get_spine_action(self, env_action: dict) -> dict:
        r"""!
        Convert environment action to a spine action dictionary.

        \param env_action Environment action.
        \return Spine action dictionary.
        """
        spine_action = {"servo": {}}
        for joint in self.model.joints:
            servo_action = {}
            for key in self.ACTION_KEYS:
                action = (
                    env_action[joint.name][key]
                    if key in env_action[joint.name]
                    else self._neutral_action[joint.name][key]
                )
                action_value = (
                    action.item()
                    if isinstance(action, np.ndarray)
                    else float(action)
                )
                servo_action[key] = clamp_and_warn(
                    action_value,
                    self._min_action[joint.name][key],
                    self._max_action[joint.name][key],
                    label=f"{joint.name}: {key}",
                )
            spine_action["servo"][joint.name] = servo_action
        return spine_action
