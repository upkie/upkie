#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

from xml.etree import ElementTree

import numpy as np

from .joints import JOINT_NAMES, NB_JOINTS


class Limits:

    def __init__(self, urdf_path: str):
        tree = ElementTree.parse(urdf_path)
        joints = [child for child in tree.getroot() if child.tag == "joint"]
        limits = [
            (joint, child)
            for joint in joints
            for child in joint
            if child.tag == "limit"
        ]
        q_max = np.full(NB_JOINTS, +np.inf)
        q_min = np.full(NB_JOINTS, -np.inf)
        tau_max = np.full(NB_JOINTS, +np.inf)
        v_max = np.full(NB_JOINTS, +np.inf)
        for joint, limit in limits:
            joint_name = joint.attrib["name"]
            joint_id = JOINT_NAMES.index(joint_name)
            q_max[joint_id] = limit.attrib["upper"]
            q_min[joint_id] = limit.attrib["lower"]
            tau_max[joint_id] = limit.attrib["effort"]
            v_max[joint_id] = limit.attrib["velocity"]
        self.q_max = q_max
        self.q_min = q_min
        self.tau_max = tau_max
        self.v_max = v_max
