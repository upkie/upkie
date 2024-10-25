#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

from typing import Optional, Set, Tuple

import numpy as np
from gymnasium import spaces

from upkie.utils.clamp import clamp_and_warn
from upkie.utils.robot_state import RobotState

from .upkie_base_env import UpkieBaseEnv


class UpkieServos(UpkieBaseEnv):
    r"""!
    Upkie with with action and observation for each servo.

    \anchor upkie_servos_description

    ### Action space

    The action space is a dictionary with one key for each servo:

    - `left_hip`: Left hip joint (qdd100)
    - `left_hip`: Left knee joint (qdd100)
    - `left_hip`: Left wheel joint (mj5208)
    - `right_hip`: Right hip joint (qdd100)
    - `right_hip`: Right knee joint (qdd100)
    - `right_hip`: Right wheel joint (mj5208)

    The value for each servo dictionary is itself a dictionary with the
    following keys:

    - `position`: Commanded joint angle \f$\theta^*\f$ in [rad] (NaN to
        disable) (required).
    - `velocity`: Commanded joint velocity \f$\dot{\theta}^*\f$ in [rad] /
        [s] (required).
    - `feedforward_torque`: Feedforward joint torque \f$\tau_{\mathit{ff}}\f$
        in [N m].
    - `kp_scale`: Scaling factor \f$k_{p}^{\mathit{scale}}\f$ applied to the
        position feedback gain, between zero and one.
    - `kd_scale`: Scaling factor \f$k_{d}^{\mathit{scale}}\f$ applied to the
        velocity feedback gain, between zero and one.
    - `maximum_torque`: Maximum joint torque \f$\tau_{\mathit{max}}\f$
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

    As with all Upkie environments, full observations from the spine (detailed
    in \ref observations) are also available in the `info` dictionary
    returned by the reset and step functions.
    """

    ACTION_KEYS: Tuple[str, str, str, str, str, str] = (
        "position",
        "velocity",
        "feedforward_torque",
        "kp_scale",
        "kd_scale",
        "maximum_torque",
    )

    ACTION_MASK: Set[str] = set()

    ## \var action_space
    ## Action space.
    action_space: spaces.dict.Dict

    ## \var observation_space
    ## Observation space.
    observation_space: spaces.dict.Dict

    ## \var version
    ## Environment version number.
    version = 4

    def __init__(
        self,
        fall_pitch: float = 1.0,
        frequency: float = 200.0,
        frequency_checks: bool = True,
        init_state: Optional[RobotState] = None,
        regulate_frequency: bool = True,
        shm_name: str = "/upkie",
        spine_config: Optional[dict] = None,
    ):
        r"""!
        Initialize environment.

        \param fall_pitch Fall pitch angle, in radians.
        \param frequency Regulated frequency of the control loop, in Hz.
        \param frequency_checks If `regulate_frequency` is set and this
            parameter is true (default), a warning is issued every time the
            control loop runs slower than the desired `frequency`. Set this
            parameter to false to disable these warnings.
        \param init_state Initial state of the robot, only used in simulation.
        \param regulate_frequency Enables loop frequency regulation.
        \param shm_name Name of shared-memory file.
        \param spine_config Additional spine configuration overriding the
            default `upkie.config.SPINE_CONFIG`. The combined configuration
            dictionary is sent to the spine at every :func:`reset`.
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

        action_space = {}
        neutral_action = {}
        max_action = {}
        min_action = {}
        servo_space = {}

        for joint in self.model.joints:
            action_space[joint.name] = spaces.Dict(
                {
                    "position": spaces.Box(
                        low=joint.limit.lower,
                        high=joint.limit.upper,
                        shape=(1,),
                        dtype=float,
                    ),
                    "velocity": spaces.Box(
                        low=-joint.limit.velocity,
                        high=+joint.limit.velocity,
                        shape=(1,),
                        dtype=float,
                    ),
                    "feedforward_torque": spaces.Box(
                        low=-joint.limit.effort,
                        high=+joint.limit.effort,
                        shape=(1,),
                        dtype=float,
                    ),
                    "kp_scale": spaces.Box(
                        low=0.0,
                        high=1.0,
                        shape=(1,),
                        dtype=float,
                    ),
                    "kd_scale": spaces.Box(
                        low=0.0,
                        high=1.0,
                        shape=(1,),
                        dtype=float,
                    ),
                    "maximum_torque": spaces.Box(
                        low=0.0,
                        high=joint.limit.effort,
                        shape=(1,),
                        dtype=float,
                    ),
                }
            )
            servo_space[joint.name] = spaces.Dict(
                {
                    "position": spaces.Box(
                        low=joint.limit.lower,
                        high=joint.limit.upper,
                        shape=(1,),
                        dtype=float,
                    ),
                    "velocity": spaces.Box(
                        low=-joint.limit.velocity,
                        high=+joint.limit.velocity,
                        shape=(1,),
                        dtype=float,
                    ),
                    "torque": spaces.Box(
                        low=-joint.limit.effort,
                        high=+joint.limit.effort,
                        shape=(1,),
                        dtype=float,
                    ),
                    "temperature": spaces.Box(
                        low=0.0,
                        high=100.0,
                        shape=(1,),
                        dtype=float,
                    ),
                    "voltage": spaces.Box(
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
                "kp_scale": 1.0,
                "kd_scale": 1.0,
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

        # gymnasium.Env: action_space
        self.action_space = spaces.Dict(action_space)

        # gymnasium.Env: observation_space
        self.observation_space = spaces.Dict(servo_space)

        # Class attributes
        self.__neutral_action = neutral_action
        self.__max_action = max_action
        self.__min_action = min_action

    def get_neutral_action(self) -> dict:
        r"""!
        Get the neutral action where servos don't move.

        \return Neutral action where servos don't move.
        """
        return self.__neutral_action.copy()

    def get_env_observation(self, spine_observation: dict):
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
                    and (not self.ACTION_MASK or key in self.ACTION_MASK)
                    else self.__neutral_action[joint.name][key]
                )
                action_value = (
                    action.item()
                    if isinstance(action, np.ndarray)
                    else float(action)
                )
                servo_action[key] = clamp_and_warn(
                    action_value,
                    self.__min_action[joint.name][key],
                    self.__max_action[joint.name][key],
                    label=f"{joint.name}: {key}",
                )
            spine_action["servo"][joint.name] = servo_action
        return spine_action

    def get_reward(self, observation: dict, action: dict) -> float:
        r"""!
        Get reward from observation and action.

        \param observation Environment observation.
        \param action Environment action.
        \return Reward.
        """
        return 1.0
