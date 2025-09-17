#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

"""Point contact data class for robot physics simulation."""

from dataclasses import dataclass

import numpy as np


@dataclass
class PointContact:
    """!
    Contact information from the robot's perspective.
    """

    ## \var link_name
    ## Name of the robot link in contact.
    link_name: str

    ## \var position_contact_in_world
    ## Position of the contact point in world coordinates.
    position_contact_in_world: np.ndarray

    ## \var force_in_world
    ## Total contact force vector in world coordinates.
    force_in_world: np.ndarray
