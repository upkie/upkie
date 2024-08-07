#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

from typing import List
from xml.etree import ElementTree

import numpy as np

from .joint import Joint
from .joint_limit import JointLimit


class Model:
    """!
    Robot model parsed from its URDF description.
    """

    ## @var joints
    ## Joints of the robot model.
    joints: List[Joint]

    def __init__(self, urdf_path: str):
        r"""!
        Constructor for the robot model wrapper.

        \param[in] urdf_path Path to the robot description.
        """
        tree = ElementTree.parse(urdf_path)
        joints = [child for child in tree.getroot() if child.tag == "joint"]
        limits = [
            (joint, child)
            for joint in joints
            for child in joint
            if child.tag == "limit"
        ]
        joints = []
        for idx, (joint, limit) in enumerate(limits):
            joint_name = joint.attrib["name"]
            joints.append(
                Joint(
                    index=idx + 1,  # starts from 1
                    idx_q=idx,
                    idx_v=idx,
                    name=joint_name,
                    limit=JointLimit(
                        lower=float(limit.attrib.get("lower", -np.inf)),
                        upper=float(limit.attrib.get("upper", +np.inf)),
                        effort=float(limit.attrib["effort"]),
                        velocity=float(limit.attrib["velocity"]),
                    ),
                )
            )
        self.joints = joints
