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
    r"""!
    Command servos by torque control.

    \anchor upkie_servo_torques_description

    ### Action space

    The action space is consists of the following targets for each joint:

    - `feedforward_torque`: feedforward joint torque \f$\tau_{\mathit{ff}}\f$
       in [N m].

    This makes the agent command joint torques directly:

    \f[
    \begin{align*}
    \tau & = \tau_{\mathit{ff}}
    \end{align*}
    \f]

    ### Observation space

    This environment has the same observation space as [UpkieServos](\ref
    upkie_servos_description).
    """

    ACTION_MASK: Set[str] = set(["feedforward_torque"])

    ## \var action_space
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
            default `upkie.config.SPINE_CONFIG`. The combined configuration
            dictionary is sent to the spine at every reset.
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
