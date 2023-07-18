#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria

"""Wheel balancing using model predictive control of an LTV system."""

import asyncio
import time
from dataclasses import dataclass

import gym
import mpacklog
import numpy as np
import upkie.envs
from ltv_mpc import MPCProblem, solve_mpc

upkie.envs.register()


@dataclass
class CartPole:

    gravity: float = 9.81  # m/s²
    length: float = 0.6  # m
    max_ground_accel: float = 10  # m/s²


def build_mpc_problem(
    horizon_duration: float, nb_timesteps: float
) -> MPCProblem:
    r"""Build MPC problem.

    @param horizon_duration Duration of the receding horizon in seconds.
    @param nb_timesteps Number of timesteps in the horizon.
    @returns MPC problem.

    - State: $x = [r\ \theta\ \dot{r}\ \dot{\theta}]$.
    - Input: $u = [\ddot{r}]$.
    """
    cart_pole = CartPole()
    T = horizon_duration / nb_timesteps
    omega = np.sqrt(cart_pole.gravity / cart_pole.length)
    g = cart_pole.gravity

    A_disc = np.array(
        [
            [1.0, 0.0, T, 0.0],
            [0.0, np.cosh(T * omega), 0.0, np.sinh(T * omega) / omega],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, omega * np.sinh(T * omega), 0.0, np.cosh(T * omega)],
        ]
    )

    B_disc = np.array(
        [
            [T ** 2 / 2.0],
            [-np.cosh(T * omega) / g + 1.0 / g],
            [T],
            [-omega * np.sinh(T * omega) / g],
        ]
    )

    ground_accel_ineq_matrix = np.vstack([np.eye(1), -np.eye(1)])
    ground_accel_ineq_vector = np.hstack(
        [+cart_pole.max_ground_accel, -cart_pole.max_ground_accel]
    )

    return MPCProblem(
        transition_state_matrix=A_disc,
        transition_input_matrix=B_disc,
        ineq_state_matrix=None,
        ineq_input_matrix=ground_accel_ineq_matrix,
        ineq_vector=ground_accel_ineq_vector,
        goal_state=np.zeros(4),
        nb_timesteps=nb_timesteps,
        terminal_cost_weight=1.0,
        stage_state_cost_weight=1e-6,
        stage_input_cost_weight=1e-3,
    )


async def balance(env: gym.Env, logger: mpacklog.AsyncLogger):
    """!
    Run proportional balancer in gym environment with logging.

    @param env Gym environment to Upkie.
    @param logger Additional logger.
    """
    observation = env.reset()  # connects to the spine
    action = np.zeros(env.action_space.shape)
    mpc_problem = build_mpc_problem(horizon_duration=1.0, nb_timesteps=12)
    for step in range(5):
        observation, reward, done, info = await env.async_step(action)
        if done:
            observation = env.reset()

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
        mpc_problem.set_initial_state(initial_state)
        plan = solve_mpc(mpc_problem, solver="quadprog")
        if plan.is_empty:
            print(f"ARGH! {plan.first_input=}")
            print(f"{action=}")
            action[0] = plan.first_input
            print(f"{action=}")
            break
        action[0] = plan.first_input
        print(f"{action=}")
        await logger.put(  # log info to be written to file later
            {
                "action": info["action"],
                "observation": info["observation"],
                "time": time.time(),
            }
        )


async def main():
    """Main function of our asyncio program."""
    logger = mpacklog.AsyncLogger("mpc_balancing.mpack")
    with gym.make("UpkieWheelsEnv-v3", frequency=200.0) as env:
        await asyncio.gather(
            balance(env, logger),
            logger.write(),  # write logs to file when there is time
        )


if __name__ == "__main__":
    asyncio.run(main())
