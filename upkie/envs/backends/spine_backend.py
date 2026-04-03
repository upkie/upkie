#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
#
## \namespace upkie.envs.backends.spine_backend
## \brief Backend connected to a simulation or real spine.

import time
from pathlib import Path
from typing import Any, Dict, Optional

import yaml

from upkie.logging import logger
from upkie.model import Model
from upkie.utils.nested_update import nested_update
from upkie.utils.raspi import on_raspi
from upkie.utils.robot_state import RobotState

from .backend import Backend
from .spine import SpineInterface


def _get_user_config_path() -> Path:
    r"""!
    Get path to the user configuration file.

    \return Path to ~/.config/upkie/config.yaml or ~/.config/upkie/config.yml,
        whichever exists (preferring .yaml).
    """
    config_dir = Path.home() / ".config" / "upkie"
    yaml_path = config_dir / "config.yaml"
    if yaml_path.exists():
        return yaml_path
    return config_dir / "config.yml"


def _load_yaml_config(config_path: Path) -> Dict[str, Any]:
    r"""!
    Load YAML configuration from file.

    \param config_path Path to the configuration file.
    \return Configuration dictionary, empty if the file doesn't exist or can't
        be loaded.
    """
    if not config_path.exists():
        return {}

    try:
        with config_path.open("r") as file:
            return yaml.safe_load(file)
    except Exception as e:
        logger.warning(f"Failed to load config from {config_path}: {e}")
        return {}


def merge_user_spine_config(
    default_config: Dict[str, Any],
    user_config: Dict[str, Any],
) -> Dict[str, Any]:
    r"""!
    Merge user configuration with default spine configuration.

    \param default_config Default spine configuration dictionary.
    \param user_config User configuration dictionary.
    \return Merged configuration with user overrides applied.
    """
    merged_config = default_config.copy()
    spine_overrides = user_config.get("spine", {})
    if spine_overrides:
        nested_update(merged_config, spine_overrides)
    return merged_config


## Default spine configuration dictionary with baseline settings.
## This dictionary contains the default configuration values for various
## spine components including bullet physics, contact detection, and odometry.
_DEFAULT_SPINE_CONFIG = {
    "bullet": {
        "gui": True,
        "reset": {
            "orientation_base_in_world": [1.0, 0.0, 0.0, 0.0],
            "position_base_in_world": [0.0, 0.0, 0.6],
        },
        "torque_control": {
            "kp": 20.0,
            "kd": 1.0,
        },
    },
    "floor_contact": {
        "upper_leg_torque_threshold": 10.0,
    },
    "wheel_contact": {
        "cutoff_period": 0.2,
        "liftoff_inertia": 0.001,
        "min_touchdown_acceleration": 2.0,
        "min_touchdown_torque": 0.015,
        "touchdown_inertia": 0.004,
    },
    "wheel_odometry": {
        "signed_radius": {
            "left_wheel": +0.05,
            "right_wheel": -0.05,
        }
    },
}


class SpineBackend(Backend):
    r"""!
    Backend connected to a simulation or real spine.

    Note that the spine backend is made to run on a single CPU thread. This is
    not a good fit for reinforcement learning, in most cases, but it is
    convenient for running exactly the same code that will be deployed to the
    real robot.
    """

    def __init__(
        self,
        shm_name: str = "/upkie",
        model: Optional[Model] = None,
        spine_config: Optional[dict] = None,
    ) -> None:
        r"""!
        Initialize spine backend.

        \param shm_name Name of shared-memory file to exchange with the spine.
        \param model Robot model. If None, defaults to the standard Upkie model
            from `upkie_description`.
        \param spine_config Additional spine configuration overriding the
            defaults. The combined configuration dictionary is sent to the
            spine at every reset.
        """
        model = model if model is not None else Model()

        merged_spine_config = merge_user_spine_config(
            default_config=_DEFAULT_SPINE_CONFIG,
            user_config=_load_yaml_config(_get_user_config_path()),
        )
        if spine_config is not None:
            nested_update(merged_spine_config, spine_config)

        # Set wheel odometry signed radius from robot model
        sign = +1.0 if model.left_wheeled else -1.0
        signed_radius = sign * model.wheel_radius
        merged_spine_config["wheel_odometry"]["signed_radius"] = {
            "left_wheel": signed_radius,
            "right_wheel": -signed_radius,
        }

        # Instance attributes
        self._spine = SpineInterface(shm_name, retries=10)
        self._spine_config = merged_spine_config

    def __del__(self):
        r"""!
        Stop the spine properly when destructing the backend instance.
        """
        try:
            self.close()
        except (KeyboardInterrupt, SystemExit):
            # Critical: ensure spine stops even if the process is interrupted
            # We don't re-raise, as exceptions from destructors are not great
            self.close()

    def close(self) -> None:
        r"""!
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
        spine_action = action.copy()
        spine_observation = self._spine.set_action(spine_action)
        return spine_observation

    def get_spine_observation(self) -> dict:
        r"""
        Get current observation from spine (used after reset).

        \return Spine observation dictionary.
        """
        return self._spine.get_observation()
