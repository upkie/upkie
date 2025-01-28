#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

from .joint_limit import JointLimit


class Joint:
    """!
    Joint properties (see also [joint limits](@ref joint-limits)).

    Extra indices in this class are provided as convenience for users familiar
    with Pinocchio.
    """

    ## \var index
    ## Index of the joint in the servo layout. Starts from 1 as in Pinocchio.
    index: int

    ## \var idx_q
    ## Index of the joint in configuration vectors.
    idx_q: int

    ## \var idx_v
    ## Index of the joint in tangent (velocity, torque) vectors.
    idx_v: int

    ## \var name
    ## Name of the joint.
    name: str

    ## \var limit
    ## Joint limit in configuration and tangent spaces.
    limit: JointLimit

    def __init__(
        self,
        index: int,
        idx_q: int,
        idx_v: int,
        name: str,
        limit: JointLimit,
    ):
        """!
        Manual constructor for joint properties.

        (We don't use a `dataclass` because Doxygen does not detect
        attribute-only declarations in Python as of version 1.9.1.)
        """
        self.index = index
        self.idx_q = idx_q
        self.idx_v = idx_v
        self.name = name
        self.limit = limit
