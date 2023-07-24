#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria

"""Wheel balancing using model predictive control of an LTV system."""

import asyncio
import time

import gymnasium as gym
import mpacklog
import numpy as np
from ltv_mpc import solve_mpc
from ltv_mpc.systems import CartPole

import upkie.envs
from upkie.utils.clamp import clamp_and_warn
from upkie.utils.spdlog import logging

upkie.envs.register()


async def balance(env: gym.Env, logger: mpacklog.AsyncLogger):
    """!
    Run proportional balancer in gym environment with logging.

    @param env Gym environment to Upkie.
    @param logger Additional logger.
    """
    cart_pole_params = CartPole.Parameters(
        length=0.4,
        max_ground_accel=10.0,
        nb_timesteps=5,
        sampling_period=0.05,
    )
    mpc_problem = CartPole.build_mpc_problem(cart_pole_params)
    cart_pole = CartPole(cart_pole_params, initial_state=np.zeros(4))
    cart_pole.init_live_plot(order="velocities")

    env.reset()  # connects to the spine
    action = np.zeros(env.action_space.shape)
    commanded_velocity = 0.0
    for step in range(500):
        (
            observation,
            reward,
            terminated,
            truncated,
            info,
        ) = await env.async_step(action)
        if terminated or truncated:
            observation, info = env.reset()

        # Unpack observation into initial MPC state
        (
            base_pitch,
            ground_position,
            base_angular_velocity,
            ground_velocity,
        ) = observation
        cart_pole.state = np.array(
            [
                ground_position,
                base_pitch,
                ground_velocity,
                base_angular_velocity,
            ]
        )
        mpc_problem.update_initial_state(cart_pole.state)

        cart_pole.goal_state = np.array(
            [
                0.0,
                0.0,
                0.0,
                0.0,
            ]
        )
        mpc_problem.update_goal_state(cart_pole.goal_state)
        plan = solve_mpc(mpc_problem, solver="proxqp")
        t = time.time()
        if plan.is_empty:
            logging.error("Solver found no solution to the MPC problem")
            logging.info("Continuing with last action")
        else:
            cart_pole.update_live_plot(plan, t, t)
            commanded_accel = plan.first_input
            commanded_velocity = clamp_and_warn(
                commanded_velocity + commanded_accel * env.dt,
                lower=-10.0,
                upper=+10.0,
                label="commanded_velocity",
            )
            action[0] = commanded_velocity
        await logger.put(  # log info to be written to file later
            {
                "action": info["action"],
                "observation": info["observation"],
                "time": t,
            }
        )
    await logger.stop()


async def main():
    """Main function of our asyncio program."""
    logger = mpacklog.AsyncLogger("mpc_balancing.mpack")
    with gym.make("UpkieWheelsEnv-v4", frequency=200.0) as env:
        await asyncio.gather(
            balance(env, logger),
            logger.write(),  # write logs to file when there is time
        )


if __name__ == "__main__":
    asyncio.run(main())
