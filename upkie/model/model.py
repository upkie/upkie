#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

from typing import List, Optional, Tuple
from xml.etree import ElementTree

import numpy as np
import upkie_description

from upkie.exceptions import ModelError
from upkie.utils.rotations import rotation_matrix_from_rpy

from .joint import Joint
from .joint_limit import JointLimit


class Model:
    """!
    Robot model parsed from its URDF description.
    """

    ## \var JOINT_NAMES
    ## List of joint names
    JOINT_NAMES: Tuple[str, str, str, str, str, str] = (
        "left_hip",
        "left_knee",
        "left_wheel",
        "right_hip",
        "right_knee",
        "right_wheel",
    )

    ## \var UPPER_LEG_JOINT_NAMES
    ## Upper-leg (hip and knee) joint names.
    UPPER_LEG_JOINT_NAMES: Tuple[str, str, str, str] = (
        "left_hip",
        "left_knee",
        "right_hip",
        "right_knee",
    )

    ## \var WHEEL_JOINT_NAMES
    ## Wheel joint names.
    WHEEL_JOINT_NAMES: Tuple[str, str] = ("left_wheel", "right_wheel")

    ## \var joints
    ## Joints of the robot model.
    joints: List[Joint]

    ## \var rotation_ars_to_world
    ## Rotation matrix from the ARS frame to the world frame.
    rotation_ars_to_world: np.ndarray

    ## \var rotation_base_to_imu
    ## Rotation matrix from the base frame to the IMU frame.
    rotation_base_to_imu: np.ndarray

    ## \var urdf_path
    ## Path to the robot description.
    urdf_path: str

    ## \var upper_leg_joints
    ## Upper-leg (hip and knee) joints.
    upper_leg_joints: Tuple[float]

    ## \var wheel_joints
    ## Wheel joints.
    wheel_joints: Tuple[float]

    def __init__(self, urdf_path: Optional[str] = None):
        r"""!
        Constructor for the robot model wrapper.

        \param[in] urdf_path Path to the robot description. If None, defaults
            to the description from `upkie_description`.
        """
        if urdf_path is None:
            urdf_path = upkie_description.URDF_PATH
        self.urdf_path = urdf_path

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

        # Redefine class attributes as instance attributes for Doxygen
        self.JOINT_NAMES = Model.JOINT_NAMES
        self.UPPER_LEG_JOINT_NAMES = Model.UPPER_LEG_JOINT_NAMES
        self.WHEEL_JOINT_NAMES = Model.WHEEL_JOINT_NAMES

        # Instance attributes
        self.joints = joints
        self.rotation_ars_to_world = np.diag([1.0, -1.0, -1.0])
        self.rotation_base_to_imu = self._parse_rotation_base_to_imu(
            joint_tags
        )
        self.upper_leg_joints = upper_leg_joints
        self.wheel_joints = wheel_joints

    @staticmethod
    def _parse_rotation_base_to_imu(joint_tags: list) -> np.ndarray:
        r"""!
        Parse IMU placement from URDF joint tags.

        \param joint_tags List of ``<joint>`` XML elements from the URDF.
        \return 3x3 rotation matrix (identity if no IMU joint is found).
        \raise ModelError If the IMU joint's parent link is not "base"
            or "torso".
        """
        for joint in joint_tags:
            child_link = joint.find("child")
            parent_link = joint.find("parent")
            if child_link is None or child_link.attrib.get("link") != "imu":
                continue
            if parent_link is None or parent_link.attrib.get("link") not in (
                "base",
                "torso",
            ):
                parent = parent_link.attrib.get("link")
                raise ModelError(
                    "IMU joint parent link should be 'base' or "
                    f"'torso', got '{parent}'"
                )
            origin = joint.find("origin")
            rpy = tuple(
                float(v) for v in origin.attrib.get("rpy", "0 0 0").split()
            )
            return rotation_matrix_from_rpy(rpy)
        return np.eye(3)
