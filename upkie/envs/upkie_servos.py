#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

from typing import Optional, Tuple

import numpy as np
import pinocchio as pin
import upkie_description
from gymnasium import spaces

from upkie.utils.clamp import clamp_and_warn
from upkie.utils.exceptions import ModelError
from upkie.utils.pinocchio import (
    box_position_limits,
    box_torque_limits,
    box_velocity_limits,
)
from upkie.utils.robot_state import RobotState

from .upkie_base_env import UpkieBaseEnv


class UpkieServos(UpkieBaseEnv):
    """!
    Upkie with with action and observation for each servo.

    ### Action space

    The action space is a dictionary with one key for each servo:

    - ``left_hip``: Left hip joint (qdd100)
    - ``left_hip``: Left knee joint (qdd100)
    - ``left_hip``: Left wheel joint (mj5208)
    - ``right_hip``: Right hip joint (qdd100)
    - ``right_hip``: Right knee joint (qdd100)
    - ``right_hip``: Right wheel joint (mj5208)

    The value for each dictionary is a dictionary with the following keys:

    - ``position``: Joint angle in [rad] (NaN to disable) (required).
    - ``velocity``: Joint velocity in [rad] / [s] (required).
    - ``feedforward_torque``: Joint torque in [N] * [m].
    - ``kp_scale``: Scaling factor applied to the position feedback gain,
        between zero and one.
    - ``kd_scale``: Scaling factor applied to the velocity feedback gain,
        between zero and one.
    - ``maximum_torque``: Maximum joint torque (feedforward + feedback) during
        the whole actuation step, in [N] * [m].

    ### Observation space

    The observation space is a dictionary with one key for each servo. The
    value for each key is a dictionary with keys:

    - ``position``: Joint angle in [rad].
    - ``velocity``: Joint velocity in [rad] / [s].
    - ``torque``: Joint torque in [N] * [m].
    - ``temperature``: Servo temperature in degree Celsius.
    - ``voltage": Power bus voltage of the servo, in [V].

    As with all Upkie environments, full observations from the spine (detailed
    in @ref observations) are also available in the ``info`` dictionary
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

    JOINT_NAMES: Tuple[str, str, str, str, str, str] = (
        "left_hip",
        "left_knee",
        "left_wheel",
        "right_hip",
        "right_knee",
        "right_wheel",
    )

    ## @var action_space
    ## Action space.
    action_space: spaces.box.Box

    ## @var observation_space
    ## Observation space.
    observation_space: spaces.box.Box

    ## @var robot
    ## Pinocchio robot wrapper.
    robot: pin.RobotWrapper

    ## @var version
    ## Environment version number.
    version = 4

    def __init__(
        self,
        fall_pitch: float = 1.0,
        frequency: float = 200.0,
        frequency_checks: bool = True,
        init_state: Optional[RobotState] = None,
        regulate_frequency: bool = True,
        shm_name: str = "/vulp",
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
            defaults from ``//config:spine.yaml``. The combined configuration
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

        robot = upkie_description.load_in_pinocchio(root_joint=None)
        model = robot.model
        q_min, q_max = box_position_limits(model)
        v_max = box_velocity_limits(model)
        tau_max = box_torque_limits(model)
        joint_names = list(model.names)[1:]
        if set(joint_names) != set(self.JOINT_NAMES):
            raise ModelError(
                "Upkie joints don't match:"
                f"expected {self.JOINT_NAMES} "
                f"got {joint_names}"
            )

        action_space = {}
        neutral_action = {}
        max_action = {}
        min_action = {}
        servo_space = {}
        for name in joint_names:
            joint = model.joints[model.getJointId(name)]
            action_space[name] = spaces.Dict(
                {
                    "position": spaces.Box(
                        low=q_min[joint.idx_q],
                        high=q_max[joint.idx_q],
                        shape=(1,),
                        dtype=float,
                    ),
                    "velocity": spaces.Box(
                        low=-v_max[joint.idx_v],
                        high=+v_max[joint.idx_v],
                        shape=(1,),
                        dtype=float,
                    ),
                    "feedforward_torque": spaces.Box(
                        low=-tau_max[joint.idx_v],
                        high=+tau_max[joint.idx_v],
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
                        high=tau_max[joint.idx_v],
                        shape=(1,),
                        dtype=float,
                    ),
                }
            )
            servo_space[name] = spaces.Dict(
                {
                    "position": spaces.Box(
                        low=q_min[joint.idx_q],
                        high=q_max[joint.idx_q],
                        shape=(1,),
                        dtype=float,
                    ),
                    "velocity": spaces.Box(
                        low=-v_max[joint.idx_v],
                        high=+v_max[joint.idx_v],
                        shape=(1,),
                        dtype=float,
                    ),
                    "torque": spaces.Box(
                        low=-tau_max[joint.idx_v],
                        high=+tau_max[joint.idx_v],
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
            neutral_action[name] = {
                "position": np.nan,
                "velocity": 0.0,
                "feedforward_torque": 0.0,
                "kp_scale": 1.0,
                "kd_scale": 1.0,
                "maximum_torque": tau_max[joint.idx_v],
            }
            max_action[name] = {
                "position": q_max[joint.idx_q],
                "velocity": v_max[joint.idx_v],
                "feedforward_torque": tau_max[joint.idx_v],
                "kp_scale": 1.0,
                "kd_scale": 1.0,
                "maximum_torque": tau_max[joint.idx_v],
            }
            min_action[name] = {
                "position": q_min[joint.idx_q],
                "velocity": -v_max[joint.idx_v],
                "feedforward_torque": -tau_max[joint.idx_v],
                "kp_scale": 0.0,
                "kd_scale": 0.0,
                "maximum_torque": 0.0,
            }

        # gymnasium.Env: action_space
        self.action_space = spaces.Dict(action_space)

        # gymnasium.Env: observation_space
        self.observation_space = spaces.Dict(servo_space)

        # Class members
        self.__neutral_action = neutral_action
        self.__max_action = max_action
        self.__min_action = min_action
        self.robot = robot

    def get_neutral_action(self) -> dict:
        """!
        Get the neutral action where servos don't move.

        @returns Neutral action where servos don't move.
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
            joint: {
                key: np.array(
                    [spine_observation["servo"][joint][key]],
                    dtype=float,
                )
                for key in self.observation_space[joint]
            }
            for joint in self.JOINT_NAMES
        }

    def get_spine_action(self, env_action: dict) -> dict:
        r"""!
        Convert environment action to a spine action dictionary.

        \param env_action Environment action.
        \return Spine action dictionary.
        """
        spine_action = {"servo": {}}
        for joint in self.JOINT_NAMES:
            servo_action = {}
            for key in self.ACTION_KEYS:
                action = (
                    env_action[joint][key]
                    if key in env_action[joint]
                    else self.__neutral_action[joint][key]
                )
                servo_action[key] = clamp_and_warn(
                    action,
                    self.__min_action[joint][key],
                    self.__max_action[joint][key],
                    label=f"{joint}: {key}",
                )
            spine_action["servo"][joint] = servo_action
        return spine_action

    def get_reward(self, observation: dict, action: dict) -> float:
        r"""!
        Get reward from observation and action.

        \param observation Environment observation.
        \param action Environment action.
        \return Reward.
        """
        return 1.0
