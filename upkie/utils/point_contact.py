#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

"""Point contact data class for robot physics simulation."""

import numpy as np


class PointContact:
    """!
    Contact information from the robot's perspective.
    """

    ## \var force_in_world
    ## Total contact force vector in world coordinates.
    force_in_world: np.ndarray

    ## \var link_name
    ## Name of the robot link in contact.
    link_name: str

    ## \var position_contact_in_world
    ## Position of the contact point in world coordinates.
    position_contact_in_world: np.ndarray

    def __init__(
        self,
        link_name: str,
        position_contact_in_world: np.ndarray,
        force_in_world: np.ndarray,
    ):
        r"""!
        Initialize point contact.

        \param force_in_world Contact force vector in the world frame.
        \param link_name Name of the robot link in contact.
        \param position_contact_in_world Position of the contact point in the
            world frame.
        """
        self.force_in_world = force_in_world
        self.link_name = link_name
        self.position_contact_in_world = position_contact_in_world

    def __repr__(self) -> str:
        """!
        String representation of the point contact.

        \return String representation showing all contact information.
        """
        return (
            f"PointContact("
            f"link_name='{self.link_name}', "
            f"position_contact_in_world={self.position_contact_in_world.tolist()}, "
            f"force_in_world={self.force_in_world.tolist()})"
        )
