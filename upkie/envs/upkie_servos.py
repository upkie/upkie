#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria
# SPDX-License-Identifier: Apache-2.0

from typing import Optional

import numpy as np
import pinocchio as pin
import upkie_description
from gymnasium import spaces
from numpy.typing import NDArray

from upkie.utils.clamp import clamp_and_warn
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

    Vectorized actions have the following structure:

    <table>
        <tr>
            <td><strong>Index</strong></td>
            <td><strong>Description</strong></td>
            </tr>
        <tr>
            <td>``[0:nq]``</td>
            <td>Joint position commands in [rad].</td>
        </tr>
        <tr>
            <td>``[nq:nq + nv]``</td>
            <td>Joint velocity commands in [rad] / [s].</td>
        </tr>
        <tr>
            <td>``[nq + nv:nq + 2 * nv]``</td>
            <td>Joint torques in [N] * [m].</td>
        </tr>
    </table>

    ### Observation space

    Vectorized observations have the following structure:

    <table>
        <tr>
            <td><strong>Index</strong></td>
            <td><strong>Description</strong></td>
            </tr>
        <tr>
            <td>``[0:nq]``</td>
            <td>Joint positions in [rad].</td>
        </tr>
        <tr>
            <td>``[nq:nq + nv]``</td>
            <td>Joint velocities in [rad] / [s].</td>
        </tr>
        <tr>
            <td>``[nq + nv:nq + 2 * nv]``</td>
            <td>Joint torques in [N] * [m].</td>
        </tr>
    </table>

    ### Attributes

    The environment has the following attributes:

    - ``robot``: Pinocchio robot wrapper.
    - ``state_max``: Maximum values for the action and observation vectors.
    - ``state_min``: Minimum values for the action and observation vectors.
    - ``version``: Environment version number.

    See also @ref envs.upkie_base_env.UpkieBaseEnv "UpkieBaseEnv" for notes on
    using this environment.
    """

    robot: pin.RobotWrapper
    version: int = 2

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
        state_dim = model.nq + 2 * model.nv
        state_max = np.hstack([q_max, v_max, tau_max])
        state_min = np.hstack([q_min, -v_max, -tau_max])
        state_max = np.float32(state_max)
        state_min = np.float32(state_min)

        # gymnasium.Env: action_space
        self.action_space = spaces.Box(
            state_min,
            state_max,
            shape=(state_dim,),
            dtype=np.float32,
        )

        # gymnasium.Env: observation_space
        observation_space = {}
        for name in servos:
            joint = model.joints[model.getJointId(name)]
            observation_space[name] = {
                "position": spaces.Box(
                    q_min[joint.idx_q],
                    q_max[joint.idx_q],
                    shape=(1,),
                    dtype=np.float32,
                ),
                "velocity": spaces.Box(
                    -v_max[joint.idx_v],
                    +v_max[joint.idx_v],
                    shape=(1,),
                    dtype=np.float32,
                ),
                "torque": spaces.Box(
                    -tau_max[joint.idx_v],
                    +tau_max[joint.idx_v],
                    shape=(1,),
                    dtype=np.float32,
                ),
            }
        self.observation_space = spaces.Dict(observation_space)

        # Class members
        self.__joints = list(robot.model.names)[1:]
        self.__last_positions = {}
        self.__servos = servos
        self.q_max = q_max
        self.q_min = q_min
        self.robot = robot
        self.tau_max = tau_max
        self.v_max = v_max

    @property
    def servos(self) -> Tuple[str, str, str, str, str, str]:
        return self.__servos

    def parse_first_observation(self, spine_observation: dict) -> None:
        """!
        Parse first observation after the spine interface is initialized.

        @param spine_observation First observation.
        """
        self.__last_positions = {
            joint: spine_observation["servo"][joint]["position"]
            for joint in self.__joints
        }

    def get_env_observation(self, spine_observation: dict):
        """!
        Extract environment observation from spine observation dictionary.

        @param spine_observation Full observation dictionary from the spine.
        @returns Environment observation.
        """
        nq, nv = self.robot.model.nq, self.robot.model.nv
        model = self.robot.model
        obs = np.empty(nq + 2 * nv, dtype=np.float32)
        for joint in self.__joints:
            i = model.getJointId(joint) - 1
            obs[i] = spine_observation["servo"][joint]["position"]
            obs[nq + i] = spine_observation["servo"][joint]["velocity"]
            obs[nq + nv + i] = spine_observation["servo"][joint]["torque"]
        return obs

    def get_spine_action(self, action: NDArray[float]) -> dict:
        """!
        Convert environment action to a spine action dictionary.

        @param action Environment action.
        @returns Spine action dictionary.
        """
        nq = self.robot.model.nq
        model = self.robot.model
        servo_action = {
            joint: {
                "position": self.__last_positions[joint],
                "velocity": 0.0,
                "torque": 0.0,
            }
            for joint in self.__joints
        }

        nq, nv = model.nq, model.nv
        q = action[:nq]
        v = action[nq : nq + nv]
        tau = action[nq + nv : nq + 2 * nv]
        for joint in self.__joints:
            i = model.getJointId(joint) - 1
            q[i] = clamp_and_warn(
                q[i],
                self.q_min[i],
                self.q_max[i],
                label=f"{joint}: position",
            )
            v[i] = clamp_and_warn(
                v[i],
                -self.v_max[i],
                self.v_max[i],
                label=f"{joint}: velocity",
            )
            tau[i] = clamp_and_warn(
                tau[i],
                -self.tau_max[i],
                self.tau_max[i],
                label=f"{joint}: torque",
            )
            servo_action[joint]["position"] = q[i]
            servo_action[joint]["velocity"] = v[i]
            servo_action[joint]["torque"] = tau[i]
            self.__last_positions[joint] = q[i]
        return {"servo": servo_action}

    def get_reward(
        self, observation: NDArray[float], action: NDArray[float]
    ) -> float:
        """!
        Get reward from observation and action.

        @param observation Observation vector.
        @param action Action vector.
        @returns Reward.
        """
        return 1.0
