#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

import gymnasium
from gymnasium import spaces

class DirectTorqueControl(gymnasium.Wrapper) :
    """!
    Wrapper to change the action space of upkie_servos to direct torque control.
    """
    def __init__(self,env):
        super().__init__(env)
        action_space = {}
        for joint in self.model.joints:
            action_space[joint.name] = spaces.Dict(
                {
                    "feedforward_torque": spaces.Box(
                        low=-joint.limit.effort,
                        high=+joint.limit.effort,
                        shape=(1,),
                        dtype=float,
                    ),
                }
            )
            # gymnasium.Env: action_space
            self.action_space = spaces.Dict(action_space)

class AdmittanceControl(gymnasium.Wrapper) :
    """!
    Wrapper to change the action space of upkie_servos to admittance control.
    This action space does not ask for a feedforward torque nor a velocity target.
    """
    def __init__(self,env):
        super().__init__(env)
        action_space = {}
        for joint in self.model.joints:
            action_space[joint.name] = spaces.Dict(
                {
                    "position": spaces.Box(
                        low=joint.limit.lower,
                        high=joint.limit.upper,
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
                }
            )
            # gymnasium.Env: action_space
            self.action_space = spaces.Dict(action_space)