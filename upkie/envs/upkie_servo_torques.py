#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

from typing import Optional, Set

from gymnasium import spaces

from upkie.utils.robot_state import RobotState

from .upkie_servos import UpkieServos


class UpkieServoTorques(UpkieServos):
    """!
    Command servos by torque control.

    Child class of UpkieServos that defines the action space as a dictionary of
    joint names with the following key :

    - `feedforward_torque`: the desired feedforward torque of the joint

    Which simplifies the control of the robot by allowing to control directly
    the torque of the joints.
    """

    ACTION_MASK: Set[str] = set(["feedforward_torque"])

    ## @var action_space
    ## Action space.
    action_space: spaces.dict.Dict

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
        action_space = {
            joint.name: spaces.Dict(
                {
                    "feedforward_torque": spaces.Box(
                        low=-joint.limit.effort,
                        high=+joint.limit.effort,
                        shape=(1,),
                        dtype=float,
                    ),
                }
            )
            for joint in self.model.joints
        }
        self.action_space = spaces.Dict(action_space)
