#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

from xml.etree import ElementTree

from .joint import Joint
from .joint_limit import JointLimit


class Model:

    def __init__(self, urdf_path: str):
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
                    index=idx + 1,
                    idx_q=idx,
                    idx_v=idx,
                    name=joint_name,
                    limit=JointLimit(
                        lower=limit.attrib["lower"],
                        upper=limit.attrib["upper"],
                        effort=limit.attrib["effort"],
                        velocity=limit.attrib["velocity"],
                    ),
                )
            )
        self.joints = joints
