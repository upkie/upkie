#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

"""Kinematic tree parsed from a URDF description."""

from collections import deque
from typing import Dict, List, Set
from xml.etree import ElementTree

import numpy as np

from upkie.exceptions import ModelError
from upkie.utils.rotations import rotation_matrix_from_rpy

from .joint import Joint
from .joint_limit import JointLimit
from .link import Link
from .se3 import SE3


class KinematicTree:
    r"""!
    Kinematic tree parsed from a URDF description.

    Computes zero-configuration SE(3) transforms for all link frames
    relative to the base (root) link, and parses joint limits.
    """

    ## \var joints
    ## Actuated joints (those with ``<limit>`` elements) parsed from the URDF.
    joints: List[Joint]

    ## \var joint_names
    ## Set of actuated joint names.
    joint_names: Set[str]

    ## \var links
    ## All links by name.
    links: Dict[str, Link]

    ## \var _transform_link_to_base
    ## Transform from each link frame to the base frame, at zero config.
    _transform_link_to_base: Dict[str, SE3]

    ## \var _root
    ## Name of the root (base) link.
    _root: str

    def __init__(self, urdf_path: str):
        r"""!
        Parse a URDF and build the kinematic tree.

        \param urdf_path Path to the URDF file.
        """
        tree = ElementTree.parse(urdf_path)
        root = tree.getroot()

        # Parse all links
        self.links = {}
        first_link_name = None
        for elem in root:
            if elem.tag == "link":
                link = Link.from_xml(elem)
                self.links[link.name] = link
                if first_link_name is None:
                    first_link_name = link.name

        if first_link_name is None:
            raise ModelError("No links found in URDF")

        # Parse all joints and build parent-child relationships
        children_of: Dict[str, List[tuple]] = {}
        child_links = set()
        joints = []

        for elem in root:
            if elem.tag != "joint":
                continue

            parent_elem = elem.find("parent")
            child_elem = elem.find("child")
            if parent_elem is None or child_elem is None:
                continue

            parent_name = parent_elem.attrib["link"]
            child_name = child_elem.attrib["link"]

            origin = elem.find("origin")
            if origin is not None:
                rpy_str = origin.attrib.get("rpy", "0 0 0")
                xyz_str = origin.attrib.get("xyz", "0 0 0")
                rpy = tuple(float(v) for v in rpy_str.split())
                xyz = np.array([float(v) for v in xyz_str.split()])
            else:
                rpy = (0.0, 0.0, 0.0)
                xyz = np.zeros(3)

            R = rotation_matrix_from_rpy(rpy)
            transform_child_to_parent = SE3(R, xyz)

            children_of.setdefault(parent_name, []).append(
                (child_name, transform_child_to_parent)
            )
            child_links.add(child_name)

            # Parse joint limits for actuated joints
            limit_elem = elem.find("limit")
            if limit_elem is not None:
                joint_name = elem.attrib["name"]
                idx = len(joints)
                joints.append(
                    Joint(
                        index=idx + 1,  # starts from 1
                        idx_q=idx,
                        idx_v=idx,
                        name=joint_name,
                        limit=JointLimit(
                            lower=float(
                                limit_elem.attrib.get("lower", -np.inf)
                            ),
                            upper=float(
                                limit_elem.attrib.get("upper", +np.inf)
                            ),
                            effort=float(limit_elem.attrib["effort"]),
                            velocity=float(limit_elem.attrib["velocity"]),
                        ),
                    )
                )

        self.joints = joints
        self.joint_names = set(joint.name for joint in joints)

        # Root is the first link in the URDF
        self._root = first_link_name

        # BFS from root to compute all transforms to base
        self._transform_link_to_base = {self._root: SE3.identity()}
        queue = deque([self._root])
        while queue:
            parent = queue.popleft()
            for child_name, transform in children_of.get(parent, []):
                # transform_child_to_base = transform_parent_to_base *
                #                           transform_child_to_parent
                self._transform_link_to_base[child_name] = (
                    self._transform_link_to_base[parent] * transform
                )
                queue.append(child_name)

    def get_transform_frame_to_base(self, frame: str) -> SE3:
        r"""!
        Get the SE3 transform from a link frame to the base frame.

        \param frame Name of the link.
        \return SE3 transform such that base_point = T * frame_point.
        \raise ModelError If the frame name is not found.
        """
        if frame not in self._transform_link_to_base:
            raise ModelError(f"Unknown frame: '{frame}'")
        return self._transform_link_to_base[frame]

    def get_transform(self, source: str, dest: str) -> SE3:
        r"""!
        Get the SE3 transform from a source frame to a destination frame.

        The returned transform T satisfies: dest_point = T * source_point.

        \param source Name of the source link frame.
        \param dest Name of the destination link frame.
        \return SE3 transform_source_to_dest.
        \raise ModelError If either frame name is not found.
        """
        transform_source_to_base = self.get_transform_frame_to_base(source)
        transform_dest_to_base = self.get_transform_frame_to_base(dest)
        return transform_dest_to_base.inverse() * transform_source_to_base

    @property
    def frame_names(self) -> List[str]:
        """List of all link frame names."""
        return list(self._transform_link_to_base.keys())
