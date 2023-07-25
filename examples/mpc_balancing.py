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


def get_target_states(
    cart_pole: CartPole, state: np.ndarray, target_vel: float
):
    """Define the reference state trajectory over the receding horizon.

    Args:
        state: Cart-pole state at the beginning of the horizon.
        target_vel: Target ground velocity in m/s.

    Returns:
        Goal state at the end of the horizon.
    """
    nx = CartPole.STATE_DIM
    T = cart_pole.sampling_period
    target_states = np.zeros((cart_pole.nb_timesteps + 1) * nx)
    for k in range(cart_pole.nb_timesteps + 1):
        target_states[k * nx] = state[0] + (k * T) * target_vel
        target_states[k * nx + 2] = target_vel
    return target_states


async def balance(env: gym.Env, logger: mpacklog.AsyncLogger):
    """!
    Run proportional balancer in gym environment with logging.

    @param env Gym environment to Upkie.
    @param logger Additional logger.
    """
    cart_pole = CartPole(
        length=0.4,
        max_ground_accel=10.0,
        nb_timesteps=5,
        sampling_period=0.05,
    )
    mpc_problem = cart_pole.build_mpc_problem(
        terminal_cost_weight=10.0,
        stage_state_cost_weight=1.0,
        stage_input_cost_weight=1e-3,
    )

    live_plot = None
    if False:
        from ltv_mpc.live_plots import CartPolePlot  # imports matplotlib

        live_plot = CartPolePlot(cart_pole, order="velocities")

    env.reset()  # connects to the spine
    action = np.zeros(env.action_space.shape)
    commanded_velocity = 0.0
    for step in range(2000):
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
        initial_state = np.array(
            [
                ground_position,
                base_pitch,
                ground_velocity,
                base_angular_velocity,
            ]
        )
        target_vel = 0.0
        target_states = get_target_states(cart_pole, initial_state, target_vel)

        mpc_problem.update_initial_state(initial_state)
        mpc_problem.update_goal_state(target_states[-CartPole.STATE_DIM :])
        mpc_problem.update_target_states(target_states[: -CartPole.STATE_DIM])

        plan = solve_mpc(mpc_problem, solver="proxqp")
        if plan.is_empty:
            logging.error("Solver found no solution to the MPC problem")
            logging.info("Continuing with last action")
        else:  # plan was found
            cart_pole.state = initial_state
            if live_plot is not None:
                t = time.time()
                live_plot.update(plan, t, initial_state, t)
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
                "time": time.time(),
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
