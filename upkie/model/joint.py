#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

from dataclasses import dataclass

from .joint_limit import JointLimit


@dataclass
class Joint:
    """!
    Joint properties.

    Extra indices in this class are provided as convenience for users familiar
    with Pinocchio.
    """

    ## @var index
    ## Index of the joint in the servo layout. Starts from 1 as in Pinocchio.
    index: int

    ## @var idx_q
    ## Index of the joint in configuration vectors.
    idx_q: int

    ## @var idx_v
    ## Index of the joint in tangent (velocity, torque) vectors.
    idx_v: int

    ## @var name
    ## Name of the joint.
    name: str

    ## @var limit
    ## Joint limit in configuration and tangent spaces.
    limit: JointLimit
