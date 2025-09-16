#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

## \namespace upkie.utils.external_force
## \brief External force data structure.

from dataclasses import dataclass
from typing import List, Union

import numpy as np


@dataclass
class ExternalForce:
    """!
    External force applied to a robot link.
    """

    ## \var force
    ## Force vector [fx, fy, fz] in Newtons.
    ## Can be provided as a list or numpy array of 3 float values.
    force: Union[List[float], np.ndarray]

    ## \var local
    ## Whether the force is in local frame (True) or world frame (False).
    ## Default is False (world frame).
    local: bool = False

    def __post_init__(self) -> None:
        r"""!
        Validate the force vector after initialization.

        Ensures that the force is a 3D vector and converts it to numpy array
        of floats for consistent handling.
        """
        # Convert to numpy array of floats
        force_array = np.array(self.force, dtype=np.float64)

        # Validate shape
        if force_array.shape != (3,):
            raise ValueError(
                f"Force must be a 3D vector, got shape {force_array.shape}"
            )

        # Store as numpy array of floats
        self.force = force_array
