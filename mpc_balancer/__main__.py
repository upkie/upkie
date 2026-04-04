#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

import gymnasium as gym
import numpy as np
import upkie_description

import upkie.envs
from upkie.controllers import JoystickGyropodController, MPCBalancer
from upkie.logging import logger
from upkie.model import Model
from upkie.utils.clamp import clamp


class Controller:
    r"""!
    Base class for wheel balancers.
    """

    ## \var joystick_controller
    ## Joystick controller for gyropod.
    joystick_controller: JoystickGyropodController

    ## \var model
    ## Robot model.
    model: Model

    ## \var mpc_balancer
    ## Internal controller for sagittal balance.
    mpc_balancer: MPCBalancer

    def __init__(
        self,
        model: Model,
        fall_pitch: float = 1.0,
        gain_scale: float = 2.0,
        leg_length: float = 0.58,
        max_ground_accel: float = 10.0,
        max_ground_velocity: float = 3.0,
        max_yaw_velocity: float = 3.0,
        turning_gain_scale: float = 2.0,
    ):
        r"""!
        Initialize balancer.

        \param fall_pitch Fall pitch threshold, in radians.
        \param gain_scale PD gain scale for hip and knee joints.
        \param leg_length Leg length in meters.
        \param max_ground_accel Maximum commanded ground acceleration in m/s².
        \param max_ground_velocity Maximum commanded ground velocity in m/s.
        \param max_yaw_velocity Maximum yaw angular velocity in rad/s.
        \param turning_gain_scale Additional gain scaling applied when turning.
        """
        self.gain_scale = clamp(gain_scale, 0.1, 2.0)
        self.joystick_controller = JoystickGyropodController()
        self.max_ground_velocity = max_ground_velocity
        self.max_yaw_velocity = max_yaw_velocity
        self.model = model
        self.mpc_balancer = MPCBalancer(
            leg_length=leg_length,
            fall_pitch=fall_pitch,
            max_ground_accel=max_ground_accel,
            max_ground_velocity=max_ground_velocity,
        )
        self.turning_gain_scale = clamp(turning_gain_scale, 0.0, 3.0)

    def run(self, frequency: float = 200.0) -> None:
        r"""!
        Run agent on a gyropod Upkie environment.

        \param frequency Control frequency in Hz.
        """

        with gym.make(
            "Upkie-Spine-Gyropod",
            frequency=frequency,
            max_ground_velocity=self.max_ground_velocity,
            max_yaw_velocity=self.max_yaw_velocity,
        ) as env:
            dt = env.unwrapped.dt
            _, info = env.reset()
            spine_observation = info["spine_observation"]

            while True:
                # Update velocity targets from joystick
                target_ground_velocity, yaw_velocity = (
                    self.joystick_controller.step(spine_observation, dt)
                )

                # MPC computes commanded ground velocity
                ground_velocity = self.mpc_balancer.step(
                    target_ground_velocity,
                    spine_observation,
                    dt,
                )

                # Update leg gain scaling based on turning probability
                set_leg_gain_scale = env.get_wrapper_attr("set_leg_gain_scale")
                turning_prob = self.joystick_controller.turning_probability
                set_leg_gain_scale(
                    self.gain_scale + self.turning_gain_scale * turning_prob
                )

                action = np.array(
                    [
                        ground_velocity,
                        yaw_velocity,
                    ]
                )
                _, _, terminated, truncated, info = env.step(action)
                spine_observation = info["spine_observation"]

                if terminated or truncated:
                    _, info = env.reset()
                    spine_observation = info["spine_observation"]
                    self.joystick_controller.reset()
                    self.mpc_balancer.reset()


if __name__ == "__main__":
    upkie.envs.register()
    model = Model(upkie_description.URDF_PATH)
    controller = Controller(model)
    try:
        controller.run()
    except KeyboardInterrupt:
        logger.info("Terminating in response to keyboard interrupt")
