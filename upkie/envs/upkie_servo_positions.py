#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

from typing import Optional, Set

import gymnasium as gym

from upkie.utils.robot_state import RobotState

from .upkie_servos import UpkieServos


class UpkieServoPositions(UpkieServos):
    r"""!
    Command servos by position control.

    \anchor upkie_servo_positions_description

    ### Action space

    The action space is consists of the following targets for each joint:

    - `position`: commanded joint angle \f$\theta^*\f$ in [rad] (NaN to
       disable) (required).
    - `kp_scale`: scaling factor \f$k_{p}^{\mathit{scale}}\f$ applied to the
       position feedback gain, between zero and one.
    - `kd_scale`: scaling factor \f$k_{d}^{\mathit{scale}}\f$ applied to the
       velocity feedback gain, between zero and one.

    This makes the agent command servo positions with velocity damping:

    \f[
    \begin{align*}
    \tau & = k_{p} k_{p}^{\mathit{scale}} (\theta^* - \theta) - k_{d}
    k_{d}^{\mathit{scale}} \dot{\theta}
    \end{align*}
    \f]

    ### Observation space

    This environment has the same observation space as
    [UpkieServos](\ref upkie_servos_description).
    """

    ACTION_MASK: Set[str] = set(["position", "kp_scale", "kd_scale"])

    ## \var action_space
    ## Action space.
    action_space: gym.spaces.dict.Dict

    def __init__(
        self,
        frequency: float = 200.0,
        frequency_checks: bool = True,
        init_state: Optional[RobotState] = None,
        regulate_frequency: bool = True,
        shm_name: str = "/upkie",
        spine_config: Optional[dict] = None,
    ):
        r"""!
        Initialize environment.

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
            frequency=frequency,
            frequency_checks=frequency_checks,
            init_state=init_state,
            regulate_frequency=regulate_frequency,
            shm_name=shm_name,
            spine_config=spine_config,
        )
        action_space = {
            joint.name: gym.spaces.Dict(
                {
                    "position": gym.spaces.Box(
                        low=joint.limit.lower,
                        high=joint.limit.upper,
                        shape=(1,),
                        dtype=float,
                    ),
                    "kp_scale": gym.spaces.Box(
                        low=0.0,
                        high=1.0,
                        shape=(1,),
                        dtype=float,
                    ),
                    "kd_scale": gym.spaces.Box(
                        low=0.0,
                        high=1.0,
                        shape=(1,),
                        dtype=float,
                    ),
                }
            )
            for joint in self.model.joints
        }
        self.action_space = gym.spaces.Dict(action_space)
