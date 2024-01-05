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
from upkie.utils.exceptions import ActionError, ModelError
from upkie.utils.pinocchio import (
    box_position_limits,
    box_torque_limits,
    box_velocity_limits,
)

from .upkie_base_env import UpkieBaseEnv


class UpkieServos(UpkieBaseEnv):

    """!
    Upkie with full observation and joint position-velocity-torque actions.

    ### Action space

    The action space is a dictionary with one key for each servo on Upkie:

    - ``left_hip``: Left hip joint (qdd100)
    - ``left_hip``: Left knee joint (qdd100)
    - ``left_hip``: Left wheel joint (mj5208)
    - ``right_hip``: Right hip joint (qdd100)
    - ``right_hip``: Right knee joint (qdd100)
    - ``right_hip``: Right wheel joint (mj5208)

    The value for each dictionary is itself a dictionary for each

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

    Obversations from this environment are full dictionary reported by the
    spine. See @ref observations.

    ### Attributes

    The environment has the following attributes:

    - ``robot``: Pinocchio robot wrapper.
    - ``version``: Environment version number.
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

    robot: pin.RobotWrapper
    version: int = 3

    def __init__(
        self,
        fall_pitch: float = 1.0,
        frequency: float = 200.0,
        shm_name: str = "/vulp",
        spine_config: Optional[dict] = None,
    ):
        """!
        Initialize environment.

        @param fall_pitch Fall pitch angle, in radians.
        @param frequency Regulated frequency of the control loop, in Hz.
        @param shm_name Name of shared-memory file.
        @param spine_config Additional spine configuration overriding the
            defaults from ``//config:spine.yaml``. The combined configuration
            dictionary is sent to the spine at every :func:`reset`.
        """
        super().__init__(
            fall_pitch=fall_pitch,
            frequency=frequency,
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
        default_action = {}
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
                        dtype=np.float32,
                    ),
                    "velocity": spaces.Box(
                        low=-v_max[joint.idx_v],
                        high=+v_max[joint.idx_v],
                        shape=(1,),
                        dtype=np.float32,
                    ),
                    "feedforward_torque": spaces.Box(
                        low=-tau_max[joint.idx_v],
                        high=+tau_max[joint.idx_v],
                        shape=(1,),
                        dtype=np.float32,
                    ),
                    "kp_scale": spaces.Box(
                        low=0.0,
                        high=1.0,
                        shape=(1,),
                        dtype=np.float32,
                    ),
                    "kd_scale": spaces.Box(
                        low=0.0,
                        high=1.0,
                        shape=(1,),
                        dtype=np.float32,
                    ),
                    "maximum_torque": spaces.Box(
                        low=0.0,
                        high=tau_max[joint.idx_v],
                        shape=(1,),
                        dtype=np.float32,
                    ),
                }
            )
            servo_space[name] = spaces.Dict(
                {
                    "position": spaces.Box(
                        low=q_min[joint.idx_q],
                        high=q_max[joint.idx_q],
                        shape=(1,),
                        dtype=np.float32,
                    ),
                    "velocity": spaces.Box(
                        low=-v_max[joint.idx_v],
                        high=+v_max[joint.idx_v],
                        shape=(1,),
                        dtype=np.float32,
                    ),
                    "torque": spaces.Box(
                        low=-tau_max[joint.idx_v],
                        high=+tau_max[joint.idx_v],
                        shape=(1,),
                        dtype=np.float32,
                    ),
                    "temperature": spaces.Box(
                        low=0.0,
                        high=100.0,
                        shape=(1,),
                        dtype=np.float32,
                    ),
                    "voltage": spaces.Box(
                        low=10.0,  # moteus min 10 V
                        high=44.0,  # moteus max 44 V
                        shape=(1,),
                        dtype=np.float32,
                    ),
                }
            )
            default_action[name] = {
                # "position": np.nan,  # no default, needs to be explicit
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
        self.observation_space = spaces.Dict(
            {
                "imu": spaces.Dict(
                    {
                        "angular_velocity": spaces.Box(
                            low=-np.inf,
                            high=np.inf,
                            shape=(3,),
                            dtype=np.float32,
                        ),
                        "linear_acceleration": spaces.Box(
                            low=-np.inf,
                            high=np.inf,
                            shape=(3,),
                            dtype=np.float32,
                        ),
                        "orientation": spaces.Box(
                            low=-1.0,
                            high=1.0,
                            shape=(4,),
                            dtype=np.float32,
                        ),
                    }
                ),
                "servo": spaces.Dict(servo_space),
                "wheel_odometry": spaces.Dict(
                    {
                        "position": spaces.Box(
                            low=-np.inf,
                            high=np.inf,
                            shape=(1,),
                            dtype=np.float32,
                        ),
                        "velocity": spaces.Box(
                            low=-np.inf,
                            high=np.inf,
                            shape=(1,),
                            dtype=np.float32,
                        ),
                    }
                ),
            }
        )

        # Class members
        self.__default_action = default_action
        self.__max_action = max_action
        self.__min_action = min_action
        self.robot = robot

    def get_default_action(self) -> dict:
        """!
        Get a default action dictionary.

        @returns Default action dictionary.
        """
        return self.__default_action.copy()

    def get_env_observation(self, spine_observation: dict):
        """!
        Extract environment observation from spine observation dictionary.

        @param spine_observation Full observation dictionary from the spine.
        @returns Environment observation.
        """
        return {
            "imu": {
                "angular_velocity": spine_observation["imu"][
                    "angular_velocity"
                ],
                "linear_acceleration": spine_observation["imu"][
                    "linear_acceleration"
                ],
                "orientation": spine_observation["imu"]["orientation"],
            },
            "servo": {
                joint: {
                    key: spine_observation["servo"][joint][key]
                    for key in self.observation_space["servo"][joint]
                }
                for joint in self.JOINT_NAMES
            },
            "wheel_odometry": {
                "position": spine_observation["wheel_odometry"]["position"],
                "velocity": spine_observation["wheel_odometry"]["velocity"],
            },
        }

    def get_spine_action(self, env_action: dict) -> dict:
        """!
        Convert environment action to a spine action dictionary.

        @param env_action Environment action.
        @returns Spine action dictionary.
        """
        spine_action = {"servo": {}}
        for joint in self.JOINT_NAMES:
            servo_action = {}
            for key in self.ACTION_KEYS:
                try:
                    action = (
                        env_action[joint][key]
                        if key in env_action[joint]
                        else self.__default_action[joint][key]
                    )
                except KeyError as key_error:
                    raise ActionError(
                        f'Missing key "{key}" required for joint "{joint}"'
                    ) from key_error
                servo_action[key] = clamp_and_warn(
                    action,
                    self.__min_action[joint][key],
                    self.__max_action[joint][key],
                    label=f"{joint}: {key}",
                )
            spine_action["servo"][joint] = servo_action
        return spine_action

    def get_reward(self, observation: dict, action: dict) -> float:
        """!
        Get reward from observation and action.

        @param observation Environment observation.
        @param action Environment action.
        @returns Reward.
        """
        return 1.0
