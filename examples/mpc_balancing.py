#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria

"""Wheel balancing using model predictive control of an LTV system."""

import gymnasium as gym
import numpy as np
from numpy.typing import NDArray
from qpmpc import solve_mpc
from qpmpc.systems import WheeledInvertedPendulum

import upkie.envs
from upkie.utils.clamp import clamp_and_warn

upkie.envs.register()


def get_target_states(
    pendulum: WheeledInvertedPendulum,
    state: NDArray[float],
    target_vel: float = 0.0,
) -> NDArray[float]:
    """Define the reference state trajectory over the receding horizon.

    Args:
        pendulum: Instance from which we read receding-horizon properties.
        state: Initial state of the wheeled inverted pendulum system.
        target_vel: Target ground velocity in m/s.

    Returns:
        Reference state trajectory over the horizon.
    """
    nx = WheeledInvertedPendulum.STATE_DIM
    target_states = np.zeros((pendulum.nb_timesteps + 1) * nx)
    for k in range(pendulum.nb_timesteps + 1):
        target_states[k * nx] = (
            state[0] + (k * pendulum.sampling_period) * target_vel
        )
        target_states[k * nx + 2] = target_vel
    return target_states


def balance(env: gym.Env, nb_env_steps: int = 10_000) -> None:
    """!
    Balancer Upkie by closed-loop MPC on its gym environment.

    @param env Gym environment to Upkie.

    @note This example rebuilds the QP problem at every step and does not
    implement hot-starting, both of which impact performance. See the MPC
    balancer agent for a more complete example.
    """
    pendulum = WheeledInvertedPendulum(
        length=0.3,
        max_ground_accel=10.0,
        nb_timesteps=50,
        sampling_period=0.02,
    )
    mpc_problem = pendulum.build_mpc_problem(
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

        theta, ground_pos, theta_dot, ground_vel = observation
        initial_state = np.array([ground_pos, theta, ground_vel, theta_dot])
        target_states = get_target_states(pendulum, initial_state)
        mpc_problem.update_initial_state(initial_state)
        mpc_problem.update_goal_state(
            target_states[-WheeledInvertedPendulum.STATE_DIM :]
        )
        mpc_problem.update_target_states(
            target_states[: -WheeledInvertedPendulum.STATE_DIM]
        )

        # NB: we solve the MPC problem "cold" here, i.e. without hot-starting.
        # This will likely take too much time to fit in a 200 Hz control loop
        # on most current computers. See the MPC balancer for a more performant
        # implementation that runs in real-time on a Raspberry Pi 4 Model B.
        plan = solve_mpc(mpc_problem, solver="proxqp")
        if not plan.is_empty:
            commanded_accel = plan.first_input
            commanded_velocity = clamp_and_warn(
                commanded_velocity + commanded_accel * env.dt / 2.0,
                lower=-1.0,
                upper=+1.0,
                label="commanded_velocity",
            )


if __name__ == "__main__":
    with gym.make("UpkieGroundVelocity-v1", frequency=200.0) as env:
        balance(env),
