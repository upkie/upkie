#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2025 Inria

from abc import ABC, abstractmethod

class Pipeline(ABC):
    r"""!
    Interface to a spine pipeline.
    """

    ## \var action_space
    ## Action space.
    action_space: object

    ## \var observation_space
    ## Observation space.
    observation_space: object

    @abstractmethod
    def get_env_observation(self, spine_observation: dict):
        r"""!
        Extract environment observation from spine observation dictionary.

        \param spine_observation Full observation dictionary from the spine.
        \return Environment observation.
        """

    @abstractmethod
    def get_neutral_action(self):
        r"""!
        Get the neutral action, typically where nothing moves.

        \return Neutral action.
        """

    @abstractmethod
    def get_spine_action(self, env_action: dict) -> dict:
        r"""!
        Convert environment action to a spine action dictionary.

        \param env_action Environment action.
        \return Spine action dictionary.
        """
