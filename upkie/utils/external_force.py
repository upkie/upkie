#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

## \namespace upkie.utils.external_force
## \brief External force data structure.

from typing import List, Union

import numpy as np


class ExternalForce:
    """!
    External force applied to a robot link.
    """

    ## \var force
    ## Force vector [fx, fy, fz] in Newtons.
    force: np.ndarray

    ## \var local
    ## Whether the force is in local frame (True) or world frame (False).
    ## Default is False (world frame).
    local: bool

    def __init__(
        self,
        force: Union[List[float], np.ndarray],
        local: bool = False,
    ):
        r"""!
        Initialize external force.

        \param force Force vector [fx, fy, fz] in Newtons.
        \param local Whether the force is in the local (True) or world (False)
            frame.
        """
        force = np.array(force, dtype=np.float64)
        if force.shape != (3,):
            raise ValueError(
                f"Force must be a 3D vector, got shape {force.shape}"
            )

        # Instance attributes
        self.force = force
        self.local = local

    def __repr__(self) -> str:
        """!
        String representation of the external force.

        \return String representation showing force vector and frame.
        """
        return f"ExternalForce(force={self.force.tolist()}, local={self.local})"
