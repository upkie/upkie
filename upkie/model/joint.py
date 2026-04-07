#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

from typing import TYPE_CHECKING, Optional

from .joint_limit import JointLimit

if TYPE_CHECKING:
    from .link import Link
    from .se3 import SE3


class Joint:
    r"""!
    Joint properties (see also [joint limits](\ref joint-limits)).

    Extra indices in this class are provided as convenience for users familiar
    with Pinocchio.
    """

    ## \var child_link
    ## Child link in the kinematic tree. Set by KinematicTree.
    child_link: Optional["Link"]

    ## \var idx_q
    ## Index of the joint in configuration vectors.
    idx_q: int

    ## \var idx_v
    ## Index of the joint in tangent (velocity, torque) vectors.
    idx_v: int

    ## \var index
    ## Index of the joint in the servo layout. Starts from 1 as in Pinocchio.
    index: int

    ## \var limit
    ## Joint limit in configuration and tangent spaces.
    limit: JointLimit

    ## \var name
    ## Name of the joint.
    name: str

    ## \var parent_link
    ## Parent link in the kinematic tree. Set by KinematicTree.
    parent_link: Optional["Link"]

    ## \var transform_child_to_parent
    ## SE3 transform from the child link frame to the parent link frame,
    ## as specified by the joint's ``<origin>`` element in the URDF.
    ## Set by KinematicTree.
    transform_child_to_parent: Optional["SE3"]

    def __init__(
        self,
        index: int,
        idx_q: int,
        idx_v: int,
        name: str,
        limit: JointLimit,
        parent_link: Optional["Link"] = None,
        child_link: Optional["Link"] = None,
        transform_child_to_parent: Optional["SE3"] = None,
    ):
        """!
        Manual constructor for joint properties.

        (We don't use a `dataclass` because Doxygen does not detect
        attribute-only declarations in Python as of version 1.9.1.)
        """
        self.child_link = child_link
        self.idx_q = idx_q
        self.idx_v = idx_v
        self.index = index
        self.limit = limit
        self.name = name
        self.parent_link = parent_link
        self.transform_child_to_parent = transform_child_to_parent
