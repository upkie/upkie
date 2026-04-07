#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

"""Link parsed from a URDF description."""

from typing import Dict, List, Optional

import numpy as np

from upkie.utils.rotations import rotation_matrix_from_rpy

from .collision_geometry import CollisionGeometry
from .se3 import SE3


class Link:
    r"""!
    Link parsed from a URDF description.
    """

    ## \var name
    ## Name of the link.
    name: str

    ## \var collision_geometries
    ## Collision geometries attached to this link.
    collision_geometries: List[CollisionGeometry]

    def __init__(
        self,
        name: str,
        collision_geometries: Optional[List[CollisionGeometry]] = None,
    ):
        r"""!
        Construct a link.

        \param name Name of the link.
        \param collision_geometries Collision geometries for this link.
        """
        self.name = name
        self.collision_geometries = (
            collision_geometries if collision_geometries is not None else []
        )

    @staticmethod
    def from_xml(link_element) -> "Link":
        r"""!
        Parse a Link from a URDF ``<link>`` XML element.

        \param link_element XML element for the link.
        \return Parsed Link instance.
        """
        name = link_element.attrib["name"]
        collision_geometries = []
        for collision in link_element.iter("collision"):
            origin_elem = collision.find("origin")
            if origin_elem is not None:
                rpy_str = origin_elem.attrib.get("rpy", "0 0 0")
                xyz_str = origin_elem.attrib.get("xyz", "0 0 0")
                rpy = tuple(float(v) for v in rpy_str.split())
                xyz = np.array([float(v) for v in xyz_str.split()])
                origin = SE3(rotation_matrix_from_rpy(rpy), xyz)
            else:
                origin = SE3.identity()

            geometry = collision.find("geometry")
            if geometry is None:
                continue

            for geom_child in geometry:
                shape = geom_child.tag
                params: Dict[str, object] = {}
                if shape == "cylinder":
                    params["radius"] = float(geom_child.attrib["radius"])
                    params["length"] = float(geom_child.attrib["length"])
                elif shape == "box":
                    params["size"] = [
                        float(v) for v in geom_child.attrib["size"].split()
                    ]
                elif shape == "sphere":
                    params["radius"] = float(geom_child.attrib["radius"])
                elif shape == "mesh":
                    params["filename"] = geom_child.attrib.get("filename", "")
                collision_geometries.append(
                    CollisionGeometry(
                        shape=shape,
                        params=params,
                        origin=origin,
                    )
                )

        return Link(name=name, collision_geometries=collision_geometries)
