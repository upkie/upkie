#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

from typing import Dict

from .se3 import SE3


class CollisionGeometry:
    r"""!
    Collision geometry for a link, parsed from URDF.
    """

    ## \var params
    ## Shape parameters, e.g. ``{"radius": 0.05, "length": 0.04}`` for a
    ## cylinder or ``{"size": [x, y, z]}`` for a box.
    params: Dict[str, object]

    ## \var origin
    ## Transform from the collision geometry frame to the link frame.
    origin: SE3

    ## \var shape
    ## Shape type: ``"box"``, ``"cylinder"``, ``"sphere"``, or ``"mesh"``.
    shape: str

    def __init__(
        self,
        shape: str,
        params: Dict[str, object],
        origin: SE3,
    ):
        r"""!
        Construct a collision geometry.

        \param shape Shape type string.
        \param params Shape parameters dictionary.
        \param origin Transform from collision frame to link frame.
        """
        self.shape = shape
        self.params = params
        self.origin = origin
