#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2024 Inria
# SPDX-License-Identifier: Apache-2.0

"""Wheeled inverted pendulum."""

from typing import Any, Optional, Tuple

import gymnasium
import numpy as np
from gymnasium import spaces
from loop_rate_limiters import RateLimiter
from numpy import cos, sin
from numpy.typing import NDArray

from upkie.exceptions import MissingOptionalDependency, UpkieRuntimeError
from upkie.model.joints import UPPER_LEG_JOINTS
from upkie.utils.clamp import clamp_and_warn
from upkie.utils.spdlog import logging

from .rewards import WheeledInvertedPendulumReward

GRAVITY: float = 9.81  # [m] / [s]²


class WheeledInvertedPendulum(gymnasium.Env):
    r"""!
    Wheeled inverted pendulum model.

    This environment helps test and debug behaviors in a perfect or
    noise-controlled setting. It has the same observation and action spaces as
    \ref upkie.envs.upkie_ground_velocity.UpkieGroundVelocity. Model
    assumptions and discretization are summed up in [this
    note](https://scaron.info/robotics/wheeled-inverted-pendulum-model.html).

    ### Action space

    The action corresponds to the ground velocity resulting from wheel
    velocities. The action vector is simply:

    <table>
        <tr>
            <td><strong>Index</strong></td>
            <td><strong>Description</strong></td>
            </tr>
        <tr>
            <td>``0``</td>
            <td>Ground velocity in [m] / [s].</td>
        </tr>
    </table>

    ### Observation space

    Vectorized observations have the following structure:

    <table>
        <tr>
            <td><strong>Index</strong></td>
            <td><strong>Description</strong></td>
        </tr>
        <tr>
            <td>0</td>
            <td>Pitch angle of the base with respect to the world vertical, in
            radians. This angle is positive when the robot leans forward.</td>
        </tr>
        <tr>
            <td>1</td>
            <td>Position of the average wheel contact point, in meters.</td>
        </tr>
        <tr>
            <td>2</td>
            <td>Body angular velocity of the base frame along its lateral axis,
            in radians per seconds.</td>
        </tr>
        <tr>
            <td>3</td>
            <td>Velocity of the average wheel contact point, in meters per
            seconds.</td>
        </tr>
    </table>
    """

    ## @var action_space
    ## Action space.
    action_space: spaces.box.Box

    ## @var dt
    ## Period of the control loop in seconds.
    dt: float

    ## @var fall_pitch
    ## Fall pitch angle, in radians.
    fall_pitch: float

    ## @var length
    ## Length of the inverted pendulum.
    length: float

    ## @var metadata
    ## Metadata of the environment containing rendering modes.
    metadata = {"render_modes": ["plot"]}

    ## @var observation_noise
    ## Vector of standard deviations for white noise added to state
    ## observations. It has the same shape and units as an observation vector.
    observation_noise: Optional[NDArray[float]]

    ## @var observation_space
    ## Observation space.
    observation_space: spaces.box.Box

    ## @var plot
    ## Optional plot used for rendering.
    plot: Optional[Any]

    ## @var render_mode
    ## Inherited from gymnasium.Env.
    render_mode: Optional[str]

    ## @var reward
    ## Reward function of the environment.
    reward: WheeledInvertedPendulumReward

    ## @var version
    ## Environment version number.
    version = 1

    def __init__(
        self,
        fall_pitch: float = 1.0,
        frequency: float = 200.0,
        frequency_checks: bool = True,
        length: float = 0.6,
        max_ground_accel: float = 10.0,
        max_ground_velocity: float = 1.0,
        observation_noise: Optional[NDArray[float]] = None,
        regulate_frequency: bool = True,
        render_mode: Optional[str] = None,
        reward: Optional[WheeledInvertedPendulumReward] = None,
    ):
        r"""!
        Initialize a new environment.

        \param fall_pitch Fall detection pitch angle, in [rad].
        \param frequency Regulated frequency of the control loop, in Hz.
        \param frequency_checks If `regulate_frequency` is set and this
            parameter is true (default), a warning is issued every time the
            control loop runs slower than the desired `frequency`. Set this
            parameter to false to disable these warnings.
        \param length Length of the pole.
        \param max_ground_velocity Maximum commanded ground velocity in [m] /
            [s].
        \param max_ground_accel  Maximum acceleration of the ground point,
            in [m] / [s]².
        \param observation_noise Vector of standard deviations for white noise
            added to state observations. It has the same shape and units as an
            observation vector.
        \param regulate_frequency Enables loop frequency regulation.
        \param render_mode Rendering mode, set to "plot" for live plotting.
        \param reward Reward function of the environment.
        """
        if (
            render_mode is not None
            and render_mode not in self.metadata["render_modes"]
        ):
            raise UpkieRuntimeError(
                f"Render mode '{render_mode}' is not in "
                f"{self.metadata['render_modes']}"
            )

        # gymnasium.Env: observation_space
        MAX_BASE_PITCH: float = np.pi
        MAX_GROUND_POSITION: float = float("inf")
        MAX_BASE_ANGULAR_VELOCITY: float = 1000.0  # rad/s
        observation_limit = np.array(
            [
                MAX_BASE_PITCH,
                MAX_GROUND_POSITION,
                MAX_BASE_ANGULAR_VELOCITY,
                max_ground_velocity,
            ],
            dtype=float,
        )
        self.observation_space = spaces.Box(
            -observation_limit,
            +observation_limit,
            shape=observation_limit.shape,
            dtype=observation_limit.dtype,
        )

        # gymnasium.Env: action_space
        action_limit = np.array([max_ground_velocity], dtype=float)
        self.action_space = spaces.Box(
            -action_limit,
            +action_limit,
            shape=action_limit.shape,
            dtype=action_limit.dtype,
        )

        reward: WheeledInvertedPendulumReward = (
            reward if reward is not None else WheeledInvertedPendulumReward()
        )

        rate = None
        dt = 1.0 / frequency
        if regulate_frequency:
            rate = RateLimiter(
                frequency,
                name=f"{self.__class__.__name__} rate limiter",
                warn=frequency_checks,
            )

        spine_observation = {
            "base_orientation": {
                "pitch": 0.0,
                "angular_velocity": np.zeros(3),
            },
            "imu": {
                "raw_angular_velocity": np.zeros(3),
                "raw_linear_acceleration": np.zeros(3),
            },
            "servo": {
                joint: {
                    "position": 0.0,
                    "velocity": 0.0,
                }
                for joint in UPPER_LEG_JOINTS
            },
            "wheel_odometry": {
                "position": 0.0,
                "velocity": 0.0,
            },
        }

        self.__accel = np.zeros(2)
        self.__max_ground_accel = max_ground_accel
        self.__max_ground_velocity = max_ground_velocity
        self.__noise = np.zeros(4)
        self.__rate = rate
        self.__regulate_frequency = regulate_frequency
        self.__spine_observation = spine_observation
        self.__state = np.zeros(4)
        self.dt = dt
        self.fall_pitch = fall_pitch
        self.length = length
        self.observation_noise = observation_noise
        self.plot = None
        self.render_mode = render_mode
        self.reward = reward

    def reset(
        self,
        *,
        seed: Optional[int] = None,
        options: Optional[dict] = None,
    ) -> Tuple[NDArray[float], dict]:
        r"""!
        Resets the spine and get an initial observation.

        \param seed Number used to initialize the environment’s internal random
            number generator.
        \param options Currently unused.
        \return
            - ``observation``: Initial vectorized observation, i.e. an element
              of the environment's ``observation_space``.
            - ``info``: Dictionary with auxiliary diagnostic information. For
              Upkie this is the full observation dictionary sent by the spine.
        """
        super().reset(seed=seed)
        self.__state = np.zeros(4)
        observation = self.__state
        info = {"spine_observation": self._get_spine_observation()}
        if self.render_mode == "plot":
            self._reset_plot()
        return observation, info

    def _reset_plot(self):
        if self.plot is None:
            try:
                from matplotlive import LivePlot

                self.plot = LivePlot(
                    timestep=self.dt,
                    duration=1.0,
                    ylim=(-0.01, 0.01),
                    ylim_right=(-0.5, 0.5),
                )
            except ImportError as exn:
                raise MissingOptionalDependency(
                    "matplotlive not found, run `pip install matplotlive`"
                ) from exn
        self.plot.reset()
        self.plot.add_left("pitch", "b-")
        self.plot.left_axis.set_ylabel(r"Pitch angle (rad)", color="b")
        self.plot.left_axis.tick_params(axis="y", labelcolor="b")
        self.plot.add_right("ground_position", "g-")
        self.plot.right_axis.set_ylabel(r"Position (m)", color="g")
        self.plot.right_axis.tick_params(axis="y", labelcolor="g")
        self.plot.redraw()

    @staticmethod
    def _integrate(x, v, a, dt):
        """Explicit second-order Euler integration in a vector space."""
        x2 = x + dt * (v + dt * (a / 2))
        v2 = v + dt * a
        return x2, v2

    def step(
        self,
        action: NDArray[float],
    ) -> Tuple[NDArray[float], float, bool, bool, dict]:
        r"""!
        Run one timestep of the environment's dynamics. When the end of the
        episode is reached, you are responsible for calling `reset()` to reset
        the environment's state.

        \param action Action from the agent.
        \return
            - ``observation``: Observation of the environment, i.e. an element
              of its ``observation_space``.
            - ``reward``: Reward returned after taking the action.
            - ``terminated``: Whether the agent reached a terminal state,
              which can be a good or a bad thing. When true, the user needs to
              call :func:`reset()`.
            - ``truncated'': Whether the episode is reaching max number of
              steps. This boolean can signal a premature end of the episode,
              i.e. before a terminal state is reached. When true, the user
              needs to call :func:`reset()`.
            - ``info``: Dictionary with auxiliary diagnostic information. For
              us this is the full observation dictionary coming from the spine.
        """
        if self.__rate is not None:
            self.__rate.sleep()  # wait until clock tick to send the action

        theta_0, r_0, thetad_0, rd_0 = self.__state
        rd_next = clamp_and_warn(
            action[0],
            -self.__max_ground_velocity,
            self.__max_ground_velocity,
            "ground_velocity",
        )
        rdd = clamp_and_warn(
            (rd_next - rd_0) / self.dt,
            -self.__max_ground_accel,
            self.__max_ground_accel,
            "ground_acceleration",
        )
        thetadd = (GRAVITY * sin(theta_0) - rdd * cos(theta_0)) / self.length
        r, rd = self._integrate(r_0, rd_0, rdd, self.dt)
        theta, thetad = self._integrate(theta_0, thetad_0, thetadd, self.dt)

        self.__state[0] = theta
        self.__state[1] = r
        self.__state[2] = thetad
        self.__state[3] = rd
        self.__accel[0] = thetadd
        self.__accel[1] = rdd

        if self.observation_noise is not None:
            self.__noise = np.normal(scale=self.observation_noise)
        if self.render_mode == "plot":
            self._render_plot()

        observation = self.__state + self.__noise
        reward = self.reward(
            pitch=theta,
            ground_position=r,
            angular_velocity=thetad,
            ground_velocity=rd,
        )
        terminated = self.detect_fall(theta)
        truncated = False
        info = {"spine_observation": self._get_spine_observation()}
        return observation, reward, terminated, truncated, info

    def detect_fall(self, pitch: float) -> bool:
        r"""!
        Detect a fall based on the base-to-world pitch angle.

        \param pitch Pitch angle of the rotation from base to world frame.
        \return True if and only if a fall is detected.
        """
        if abs(pitch) > self.fall_pitch:
            logging.warning(
                "Fall detected (pitch=%.2f rad, fall_pitch=%.2f rad)",
                abs(pitch),
                self.fall_pitch,
            )
            return True
        return False

    def render(self):
        r"""!
        Render state to a live plot.
        """
        if self.render_mode == "plot":
            return self._render_plot()

    def _render_plot(self):
        self.plot.send("pitch", self.__state[0])
        self.plot.send("ground_position", self.__state[1])
        self.plot.update()

    def _get_spine_observation(self):
        obs = self.__spine_observation  # reference, not a copy
        obs["base_orientation"]["angular_velocity"][1] = self.__state[2]
        obs["base_orientation"]["pitch"] = self.__state[0]

        # Assumes the y-axis of the IMU is the same as that of the base frame
        obs["imu"]["raw_angular_velocity"][1] = self.__state[3]
        # obs["imu"]["raw_linear_acceleration"]

        obs["wheel_odometry"]["position"] = self.__state[1]
        obs["wheel_odometry"]["velocity"] = self.__state[3]
        return obs
