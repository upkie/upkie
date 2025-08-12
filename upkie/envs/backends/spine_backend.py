#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

import time
from typing import Optional

import upkie.config
from upkie.spine import SpineInterface
from upkie.utils.nested_update import nested_update
from upkie.utils.raspi import on_raspi
from upkie.utils.robot_state import RobotState

from .backend import Backend


class SpineBackend(Backend):
    r"""!
    Backend connected to a simulation or real spine.
    """

    def __init__(
        self,
        shm_name: str = "/upkie",
        spine_config: Optional[dict] = None,
    ) -> None:
        r"""!
        Initialize spine backend.

        \param shm_name Name of shared-memory file to exchange with the spine.
        \param spine_config Additional spine configuration overriding the
            default `upkie.config.SPINE_CONFIG`. The combined configuration
            dictionary is sent to the spine at every reset.
        """
        merged_spine_config = upkie.config.SPINE_CONFIG.copy()
        if spine_config is not None:
            nested_update(merged_spine_config, spine_config)

        # Class attributes
        self.__bullet_action = {}
        self._spine = SpineInterface(shm_name, retries=10)
        self._spine_config = merged_spine_config

    def __del__(self):
        """!
        Stop the spine when deleting the backend instance.
        """
        self.close()

    def close(self) -> None:
        """!
        Stop the spine properly.
        """
        if hasattr(self, "_spine"):  # in case SpineError was raised in ctor
            self._spine.stop()

    def reset(self, init_state: Optional[RobotState] = None) -> dict:
        r"""!
        Reset the spine and get an initial observation.

        \param init_state Initial state of the robot (for simulation spines).
        \return Initial spine observation dictionary.
        """
        self._spine.stop()
        if on_raspi():
            # If we start the spine right after it stops, it can cause an issue
            # where some moteus controllers fault with a (blinking red +
            # stationary green) error code. For now we assume resetting on the
            # real robot can take time.
            time.sleep(1.0)

        if init_state is not None:
            self._reset_robot_state(init_state)

        spine_observation = self._spine.start(self._spine_config)
        return spine_observation

    def _reset_robot_state(self, init_state: RobotState):
        r"""!
        Configure spine config with initial state for simulation.

        \param init_state Initial state of the robot.
        """
        orientation_quat = init_state.orientation_base_in_world.as_quat()
        qx, qy, qz, qw = orientation_quat
        orientation_quat = [qw, qx, qy, qz]  # Convert to [w, x, y, z] format
        position = init_state.position_base_in_world
        linear_velocity = init_state.linear_velocity_base_to_world_in_world
        omega = init_state.angular_velocity_base_in_base

        bullet_config = self._spine_config["bullet"]
        reset = bullet_config["reset"]
        reset["orientation_base_in_world"] = orientation_quat
        reset["position_base_in_world"] = position
        reset["linear_velocity_base_to_world_in_world"] = linear_velocity
        reset["angular_velocity_base_in_base"] = omega
        reset["joint_configuration"] = init_state.joint_configuration

    def step(self, action: dict) -> dict:
        r"""!
        Apply action and step the spine.

        \param action Action dictionary in spine format.
        \return Spine observation dictionary after the step.
        """
        # Prepare spine action
        spine_action = action.copy()
        if self.__bullet_action:
            spine_action["bullet"] = {}
            spine_action["bullet"].update(self.__bullet_action)
            self.__bullet_action.clear()

        # Send action to and get observation from the spine
        spine_observation = self._spine.set_action(spine_action)
        return spine_observation

    def get_spine_observation(self) -> dict:
        """Get current observation from spine (used after reset)."""
        return self._spine.get_observation()

    def get_bullet_action(self) -> dict:
        r"""!
        Get the Bullet action that will be applied at next step.

        \return Upcoming simulator action.
        """
        return self.__bullet_action

    def set_bullet_action(self, bullet_action: dict) -> None:
        r"""!
        Prepare for the next step an extra action for the Bullet spine.

        This extra action can be for instance a set of external forces applied
        to some robot bodies.

        \param bullet_action Action dictionary processed by the Bullet spine.
        """
        self.__bullet_action = bullet_action.copy()
