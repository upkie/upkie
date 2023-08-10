#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria

"""Wheel balancing using model predictive control of an LTV system."""

import os
import sys

import gymnasium as gym
import numpy as np
from ltv_mpc import solve_mpc
from ltv_mpc.systems import CartPole

import upkie.envs
from upkie.utils.clamp import clamp_and_warn
from upkie.utils.raspi import on_raspi
from upkie.utils.spdlog import logging

upkie.envs.register()


def get_target_states(
    sampling_period: float,
    nb_timesteps: int,
    state: np.ndarray,
    target_vel: float = 0.0,
):
    """Define the reference state trajectory over the receding horizon.

    Args:
        sampling_period: Duration of a timestep in the receding horizon.
        nb_timesteps: Number of receding-horizon timesteps.
        target_vel: Target ground velocity in m/s.

    Returns:
        Goal state at the end of the horizon.
    """
    nx = CartPole.STATE_DIM
    target_states = np.zeros((nb_timesteps + 1) * nx)
    for k in range(nb_timesteps + 1):
        target_states[k * nx] = state[0] + (k * sampling_period) * target_vel
        target_states[k * nx + 2] = target_vel
    return target_states


def balance(env: gym.Env, nb_env_steps: int = 10_000):
    """!
    Balancer Upkie by closed-loop MPC on its gym environment.

    @param env Gym environment to Upkie.

    @note This example rebuilds the QP problem at every step and does not
    implement hot-starting, both of which impact performance. See the MPC
    balancer agent for a more complete example.
    """
    cart_pole = CartPole(
        length=0.3,
        max_ground_accel=10.0,
        nb_timesteps=50,
        sampling_period=0.02,
    )

    mpc_problem = cart_pole.build_mpc_problem(
        terminal_cost_weight=1.0,
        stage_state_cost_weight=1e-2,
        stage_input_cost_weight=1e-3,
    )

    env.reset()  # connects to the spine
    commanded_velocity = 0.0
    action = np.zeros(env.action_space.shape)
    for step in range(nb_env_steps):
        action[0] = commanded_velocity
        observation, _, terminated, truncated, info = env.step(action)
        if terminated or truncated:
            observation, info = env.reset()
            commanded_velocity = 0.0

        # Unpack observation into initial MPC state
        theta, ground_pos, theta_dot, ground_vel = observation
        initial_state = np.array([ground_pos, theta, ground_vel, theta_dot])
        target_states = get_target_states(
            cart_pole.sampling_period, cart_pole.nb_timesteps, initial_state
        )

        mpc_problem.update_initial_state(initial_state)
        mpc_problem.update_goal_state(target_states[-CartPole.STATE_DIM :])
        mpc_problem.update_target_states(target_states[: -CartPole.STATE_DIM])

        plan = solve_mpc(mpc_problem, solver="proxqp")
        if plan.is_empty:
            logging.error("Solver found no solution to the MPC problem")
            logging.info("Continuing with previous action")
        else:  # plan was found
            commanded_accel = plan.first_input
            commanded_velocity = clamp_and_warn(
                commanded_velocity + commanded_accel * env.dt / 2.0,
                lower=-1.0,
                upper=+1.0,
                label="commanded_velocity",
            )


if __name__ == "__main__":
    # TODO(scaron): move to a function in utils.raspi
    if on_raspi() and os.geteuid() != 0:
        print("Re-running as root so that we can set CPU affinity")
        args = ["sudo", "-E", sys.executable] + sys.argv + [os.environ]
        os.execlpe("sudo", *args)
    if on_raspi():
        CPUID = 3
        os.sched_setaffinity(0, {CPUID})

    with gym.make("UpkieWheelsEnv-v4", frequency=200.0) as env:
        balance(env),
