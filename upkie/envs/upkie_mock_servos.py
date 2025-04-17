#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

from pathlib import Path
from typing import Optional, Tuple

import numpy as np

from upkie.utils.joystick import Joystick
from upkie.utils.robot_state import RobotState

from .upkie_servos import UpkieServos


class UpkieMockServos(UpkieServos):
    r"""!
    Upkie servo environment that mimicks commanding servos perfectly.

    See [UpkieServos](\ref upkie_servos_description) for a description of the
    action and observation spaces of servo environments.
    """

    def __init__(
        self,
        frequency: Optional[float] = 200.0,
        frequency_checks: bool = True,
        init_state: Optional[RobotState] = None,
        regulate_frequency: bool = True,
    ) -> None:
        r"""!
        Initialize environment.

        \param frequency Regulated frequency of the control loop, in Hz. Can be
            prescribed even when `regulate_frequency` is unset, in which case
            `self.dt` will be defined but the loop frequency will not be
            regulated.
        \param frequency_checks If `regulate_frequency` is set and this
            parameter is true (default), a warning is issued every time the
            control loop runs slower than the desired `frequency`. Set this
            parameter to false to disable these warnings.
        \param init_state Initial state of the robot, only used in simulation.
        \param regulate_frequency If set (default), the environment will
            regulate the control loop frequency to the value prescribed in
            `frequency`.

        \throw SpineError If the spine did not respond after the prescribed
            number of trials.
        """
        super().__init__(
            frequency=frequency,
            frequency_checks=frequency_checks,
            init_state=init_state,
            regulate_frequency=regulate_frequency,
        )

        spine_observation = {
            "base_orientation": {
                "pitch": 0.1,
                "angular_velocity": [-2e-3, 3e2, 1e-8],
                "linear_velocity": [1e3, 2e2, 3e1],
            },
            "imu": {
                "orientation": [1.0, 0.0, 0.0, 0.0],
                "angular_velocity": [0.0, 0.0, 0.0],
                "linear_acceleration": [0.0, 0.0, 0.0],
            },
            "number": 0,
            "servo": {
                f"{side}_{joint}": {
                    "position": 0.0,
                    "velocity": 0.0,
                    "torque": 0.0,
                    "temperature": 42.0,
                    "voltage": 18.0,
                }
                for side in ("left", "right")
                for joint in ("hip", "knee", "wheel")
            },
            "wheel_odometry": {
                "position": 0.0,
                "velocity": 0.0,
            },
        }

        js_path = Path("/dev/input/js0")
        joystick = Joystick(js_path) if js_path.exists() else None

        self.__spine_observation = spine_observation
        self.joystick = joystick

    def reset(
        self,
        seed: Optional[int] = None,
        options: Optional[dict] = None,
    ) -> Tuple[dict, dict]:
        r"""!
        Resets the spine and get an initial observation.

        \param seed Number used to initialize the environment’s internal random
            number generator.
        \param options Currently unused.
        \return
            - `observation`: Initial vectorized observation, i.e. an element
              of the environment's `observation_space`.
            - `info`: Dictionary with auxiliary diagnostic information. For
              Upkie this is the full observation dictionary sent by the spine.
        """
        super().reset(seed=seed)
        self._spine.stop()
        self.__reset_rate()
        self.__reset_init_state()
        spine_observation = self._spine.start(self._spine_config)
        observation = self.__get_observation(spine_observation)
        info = {"spine_observation": spine_observation}
        return observation, info

    def step(
        self,
        action: np.ndarray,
    ) -> Tuple[np.ndarray, float, bool, bool, dict]:
        r"""!
        Run one timestep of the environment's dynamics.

        When the end of the episode is reached, you are responsible for calling
        `reset()` to reset the environment's state.

        \param action Action from the agent.
        \return
            - `observation`: Observation of the environment, i.e. an element
              of its `observation_space`.
            - `reward`: Reward returned after taking the action.
            - `terminated`: Whether the agent reached a terminal state,
              which may be a good or a bad thing. When true, the user needs to
              call `reset()`.
            - `truncated`: Whether the episode is reaching max number of
              steps. This boolean can signal a premature end of the episode,
              i.e. before a terminal state is reached. When true, the user
              needs to call `reset()`.
            - `info`: Dictionary with additional information, reporting in
              particular the full observation dictionary coming from the spine.
        """
        if self.__regulate_frequency:
            self.__rate.sleep()  # wait until clock tick to send the action
            self.log("rate", {"slack": self.__rate.slack})

        # Prepare spine action
        spine_action = self.__get_spine_action(action)
        for key in ("bullet", "log"):
            if not self.__bonus_action[key]:
                continue
            spine_action[key] = {}
            spine_action[key].update(self.__bonus_action[key])
            self.__bonus_action[key].clear()

        # Send action to and get observation from the spine
        spine_observation = self._spine.set_action(spine_action)

        # Process spine observation
        observation = self.__get_observation(spine_observation)
        reward = 1.0  # ready for e.g. an ObservationBasedReward wrapper
        terminated = False
        truncated = False  # will be handled by e.g. a TimeLimit wrapper
        info = {"spine_observation": spine_observation}
        return observation, reward, terminated, truncated, info

    def __get_observation(self, spine_observation: dict):
        r"""!
        Extract environment observation from spine observation dictionary.

        \param spine_observation Full observation dictionary from the spine.
        \return Environment observation.
        """
        # If creating a new object turns out to be too slow we can switch to
        # updating in-place.
        return {
            joint.name: {
                key: np.array(
                    [spine_observation["servo"][joint.name][key]],
                    dtype=float,
                )
                for key in self.observation_space[joint.name]
            }
            for joint in self.model.joints
        }

    def __get_spine_action(self, env_action: dict) -> dict:
        r"""!
        Convert environment action to a spine action dictionary.

        \param env_action Environment action.
        \return Spine action dictionary.
        """
        spine_action = {"servo": {}}
        for joint in self.model.joints:
            servo_action = {}
            for key in self.ACTION_KEYS:
                action = (
                    env_action[joint.name][key]
                    if key in env_action[joint.name]
                    and (not self.ACTION_MASK or key in self.ACTION_MASK)
                    else self.__neutral_action[joint.name][key]
                )
                action_value = (
                    action.item()
                    if isinstance(action, np.ndarray)
                    else float(action)
                )
                servo_action[key] = clamp_and_warn(
                    action_value,
                    self.__min_action[joint.name][key],
                    self.__max_action[joint.name][key],
                    label=f"{joint.name}: {key}",
                )
            spine_action["servo"][joint.name] = servo_action
        return spine_action

    def log(self, name: str, entry: Any) -> None:
        r"""!
        Log a new entry to the "log" key of the action dictionary.

        \param name Name of the entry.
        \param entry Dictionary to log along with the actual action.
        """
        if isinstance(entry, dict):
            self.__bonus_action["log"][name] = entry.copy()
        else:  # logging values directly
            self.__bonus_action["log"][name] = entry

    def get_bullet_action(self) -> dict:
        r"""!
        Get the Bullet action that will be applied at next step.

        \return Upcoming simulator action.
        """
        return self.__bonus_action["bullet"]

    def set_bullet_action(self, bullet_action: dict) -> None:
        r"""!
        Prepare for the next step an extra action for the Bullet spine.

        This extra action can be for instance a set of external forces applied
        to some robot bodies.

        \param bullet_action Action dictionary processed by the Bullet spine.
        """
        self.__bonus_action["bullet"] = bullet_action.copy()
