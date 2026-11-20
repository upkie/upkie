# SPDX-License-Identifier: Apache-2.0
#
# /// script
# dependencies = ["upkie", "proxsuite", "pybullet", "qpmpc"]
# ///

"""Balancing with the wheels by model predictive control in PyBullet."""

import argparse

import gymnasium as gym
import numpy as np

import upkie.envs
from upkie.controllers import MPCBalancer

upkie.envs.register()

NB_STEPS = 5_000


def parse_command_line_arguments() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--robot",
        help="Robot to simulate (default: upkie)",
        choices=["upkie", "cookie"],
        default="upkie",
    )
    parser.add_argument(
        "--velocity",
        help="Forward velocity, in m/s",
        default=0.3,
        type=float,
    )
    return parser.parse_args()


def main(robot: str, target_velocity: float):
    """Track the target ground velocity."""
    # Leg length is a key parameter of the model predictive controller. Start
    # from the leg length of the wheeled biped, but don't hesitate to tune this
    # value afterwards for better performance on your real robot.
    mpc_balancer = MPCBalancer(leg_length=0.58)

    print(f"\nStarting balancing a {robot} by MPC in PyBullet...")
    with gym.make(
        f"{robot}-PyBullet-Pendulum",
        frequency=200.0,
        frequency_checks=False,
        gui=True,
        nb_substeps=5,
    ) as env:
        _, info = env.reset()  # connects to the spine
        action = np.zeros(env.action_space.shape)

        print("\n--\n")
        print(f"Going forward for {NB_STEPS} steps...")
        print(f"Target ground velocity: {target_velocity} m/s")
        print("Press Ctrl+C to stop early.\n")

        for step in range(NB_STEPS):
            action[0] = mpc_balancer.step(
                target_ground_velocity=target_velocity,
                spine_observation=info["spine_observation"],
                dt=env.unwrapped.dt,
            )
            _, _, terminated, truncated, info = env.step(action)

            if (step + 1) % 500 == 0:
                ground_pos = info["spine_observation"]["wheel_odometry"][
                    "position"
                ]
                base_pitch = info["spine_observation"]["base_orientation"][
                    "pitch"
                ]
                print(
                    f"Step {step + 1:5d}: "
                    f"Ground position = {ground_pos:6.3f} m "
                    f"at t = {step * env.unwrapped.dt:4.1f} s, "
                    f"Base pitch = {base_pitch:5.3f} rad"
                )

            if terminated or truncated:
                print("Episode ended, resetting...")
                _, info = env.reset()

        print("Example completed.")


if __name__ == "__main__":
    args = parse_command_line_arguments()
    try:
        main(
            robot=args.robot.capitalize(),
            target_velocity=args.velocity,
        )
    except KeyboardInterrupt:
        print("Terminating in response to keyboard interrupt")
