#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

from typing import List, Tuple
from xml.etree import ElementTree

import numpy as np

from .joint import Joint
from .joint_limit import JointLimit


class Model:
    """!
    Robot model parsed from its URDF description.
    """

    ## \var joints
    ## Joints of the robot model.
    joints: List[Joint]

    ## \var rotation_ars_to_world
    ## Rotation matrix from the ARS frame to the world frame.
    rotation_ars_to_world: np.ndarray

    ## \var rotation_base_to_imu
    ## Rotation matrix from the base frame to the IMU frame.
    rotation_base_to_imu: np.ndarray

    ## \var upper_leg_joints
    ## Upper-leg (hip and knee) joints.
    upper_leg_joints: Tuple[float]

    ## \var wheel_joints
    ## Wheel joints.
    wheel_joints: Tuple[float]

    ## JOINT_NAMES
    ## List of joint names
    JOINT_NAMES: Tuple[str, str, str, str, str, str] = (
        "left_hip",
        "left_knee",
        "left_wheel",
        "right_hip",
        "right_knee",
        "right_wheel",
    )

    ## UPPER_LEG_JOINT_NAMES
    ## Upper-leg (hip and knee) joint names.
    UPPER_LEG_JOINT_NAMES: Tuple[str, str, str, str] = (
        "left_hip",
        "left_knee",
        "right_hip",
        "right_knee",
    )

    ## WHEEL_JOINT_NAMES
    ## Wheel joint names.
    WHEEL_JOINT_NAMES: Tuple[str, str] = ("left_wheel", "right_wheel")

    def __init__(self, urdf_path: str):
        r"""!
        Constructor for the robot model wrapper.

        \param[in] urdf_path Path to the robot description.
        """
        tree = ElementTree.parse(urdf_path)
        joint_tags = [
            child for child in tree.getroot() if child.tag == "joint"
        ]
        limits = [
            (joint, child)
            for joint in joint_tags
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
        upper_leg_joints = tuple(
            joint
            for joint in joints
            if joint.name in self.UPPER_LEG_JOINT_NAMES
        )
        wheel_joints = tuple(
            joint for joint in joints if joint.name in self.WHEEL_JOINT_NAMES
        )
        self.joints = joints
        self.rotation_ars_to_world = np.diag([1.0, -1.0, -1.0])
        self.rotation_base_to_imu = np.diag([-1.0, 1.0, -1.0])
        self.upper_leg_joints = upper_leg_joints
        self.wheel_joints = wheel_joints
