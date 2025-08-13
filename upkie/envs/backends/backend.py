#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

## \namespace upkie.envs.backends.backend
## \brief Interface to a backend for Upkie environments.

from abc import ABC, abstractmethod

from upkie.utils.robot_state import RobotState


class Backend(ABC):
    r"""!
    Interface to a backend for Upkie environments.

    Backends handle the low-level interface to simulators or spines, providing
    methods to get observations, apply actions, and manage the
    simulation/spine state.
    """

    @abstractmethod
    def get_spine_observation(self) -> dict:
        r"""!
        Get observation in spine format from the backend.

        \return Spine observation dictionary.
        """

    @abstractmethod
    def reset(self, init_state: RobotState) -> dict:
        r"""!
        Reset the backend to an initial state.

        \param init_state Initial robot state.
        \return Initial spine observation dictionary.
        """

    @abstractmethod
    def step(self, action: dict) -> dict:
        r"""!
        Apply action and step the backend.

        \param action Action dictionary in spine format.
        \return Spine observation dictionary after stepping.
        """

    @abstractmethod
    def close(self) -> None:
        r"""!
        Clean up backend resources.
        """
