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
        leg_length: float = 0.58,
        max_ground_accel: float = 10.0,
        max_ground_velocity: float = 3.0,
        turning_deadband: float = 0.3,
        turning_decision_time: float = 0.2,
        max_linear_velocity: float = 1.5,
        max_linear_accel: float = 1.2,
        max_yaw_velocity: float = 1.0,
        max_yaw_accel: float = 10.0,
    ):
        r"""!
        Initialize balancer.

        \param fall_pitch Fall pitch threshold, in radians.
        \param leg_length Leg length in meters.
        \param turning_deadband Joystick axis value between 0.0 and 1.0 below
            which legs stiffen but the turning motion doesn't start.
        \param turning_decision_time Minimum duration in seconds for the
            turning probability to switch from zero to one and conversely.
        \param max_ground_accel Maximum commanded ground acceleration.
        \param max_ground_velocity Maximum commanded ground velocity.
        \param max_linear_accel Maximum acceleration for the ground position
            target, in m/s². Does not affect the commanded ground velocity.
        \param max_linear_velocity Maximum velocity for the ground position
            target, in m/s. Indirectly affects the commanded ground velocity.
        \param max_yaw_accel Maximum yaw angular acceleration in rad/s².
        \param max_yaw_velocity Maximum yaw angular velocity in rad/s.
        """
        assert 0.0 <= turning_deadband <= 1.0
        mpc_balancer = MPCBalancer(
            leg_length=leg_length,
            fall_pitch=fall_pitch,
            max_ground_accel=max_ground_accel,
            max_ground_velocity=max_ground_velocity,
        )
        self.model = model
        self.mpc_balancer = mpc_balancer
        self.joystick_controller = JoystickGyropodController(
            turning_deadband=turning_deadband,
            turning_decision_time=turning_decision_time,
            max_linear_velocity=max_linear_velocity,
            max_linear_accel=max_linear_accel,
            max_yaw_velocity=max_yaw_velocity,
            max_yaw_accel=max_yaw_accel,
        )

    def cycle(self, observation: dict, dt: float) -> dict:
        r"""!
        Compute a new ground velocity.

        \param observation Latest observation.
        \param dt Duration in seconds until the next cycle.
        \return New ground velocity, in m/s.
        """
        target_ground_velocity, target_yaw_velocity = (
            self.joystick_controller.step(observation, dt)
        )

        ground_velocity = self.mpc_balancer.step(
            target_ground_velocity, observation, dt
        )

        # Sagittal translation
        wheel_velocity = ground_velocity / self.model.wheel_radius
        left_sign: float = 1.0 if self.model.left_wheeled else -1.0
        right_sign = -left_sign
        left_wheel_velocity = left_sign * wheel_velocity
        right_wheel_velocity = right_sign * wheel_velocity

        # Yaw rotation
        delta = observation["height_controller"]["position_right_in_left"]
        contact_radius = 0.5 * np.linalg.norm(delta)
        yaw_to_wheel = left_sign * contact_radius / self.model.wheel_radius
        left_wheel_velocity += yaw_to_wheel * target_yaw_velocity
        right_wheel_velocity += yaw_to_wheel * target_yaw_velocity

        servo_action = {
            "left_wheel": {
                "position": np.nan,
                "velocity": left_wheel_velocity,
            },
            "right_wheel": {
                "position": np.nan,
                "velocity": right_wheel_velocity,
            },
        }
        return {"servo": servo_action}

    def run(
        self,
        frequency: float = 200.0,
        gain_scale: float = 2.0,
        turning_gain_scale: float = 2.0,
    ) -> None:
        r"""!
        Run agent using a gyropod environment.

        \param frequency Control frequency in Hz.
        \param gain_scale PD gain scale for hip and knee joints.
        \param turning_gain_scale Additional gain scaling applied when turning.
        """
        upkie.envs.register()

        gain_scale = clamp(gain_scale, 0.1, 2.0)
        dt = 1.0 / frequency
        max_linear_velocity = (
            1.5 * self.joystick_controller.max_linear_velocity
        )
        max_yaw_velocity = 1.5 * self.joystick_controller.max_yaw_velocity

        with gym.make(
            "Upkie-Spine-Gyropod",
            frequency=frequency,
            max_ground_velocity=max_linear_velocity,
            max_yaw_velocity=max_yaw_velocity,
        ) as env:
            _, info = env.reset()
            spine_observation = info["spine_observation"]

            while True:
                # Update velocity targets from joystick
                target_ground_velocity, target_yaw_velocity = (
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
                    gain_scale + turning_gain_scale * turning_prob
                )

                action = np.array(
                    [
                        ground_velocity,
                        target_yaw_velocity,
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
    model = Model(upkie_description.URDF_PATH)

    logger.info(f"Wheel radius: {model.wheel_radius} m")

    controller = Controller(model)
    logger.info(
        f"Max. remote-control velocity: "
        f"{controller.joystick_controller.max_linear_velocity} m/s"
    )
    try:
        controller.run()
    except KeyboardInterrupt:
        logger.info("Terminating in response to keyboard interrupt")
