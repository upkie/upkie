# SPDX-License-Identifier: Apache-2.0
#
# /// script
# dependencies = ["upkie", "proxsuite", "pybullet", "qpmpc"]
# ///

"""Crouch up and down while balancing in place in PyBullet."""

import argparse

import gymnasium as gym
import numpy as np

import upkie.envs
from upkie.model import Model

upkie.envs.register()

## Gain scale of the hip and knee servos. Position-controlled legs sag under
## the weight of the robot, the more so with low gains. Cookie is heavier
## than Upkie, so we give it stiffer gains.
LEG_GAIN_SCALES = {
    "Upkie": 2.0,
    "Cookie": 5.0,
}

## Margin from hip and knee joint limits at the lowest crouch, in radians.
## Upkie's ankles touch the ground well before its joints reach their limits,
## so it gets a larger margin.
JOINT_LIMIT_MARGINS = {
    "Upkie": 0.7,
    "Cookie": 0.05,
}

## Constant ground velocity command, in m/s. Crouching brings the knees, and
## with them the center of mass, forward of the wheels. The balancer then
## leans back and drifts forward to stay balanced: a constant backward
## velocity command roughly compensates for this drift.
COMPENSATION_VELOCITIES = {
    "Upkie": -0.4,
    "Cookie": -0.8,
}

## Period of the up-and-down crouching motion, in seconds
PERIOD = 8.0


def compute_max_crouching_height(model: Model, margin: float) -> float:
    """Compute the crouching height where legs come close to joint limits.

    Parameters
    ----------
    model : Model
        Robot model.
    margin : float
        Margin from hip and knee joint limits, in radians.

    Returns
    -------
    float
        Crouching height, in meters, at which a hip or knee joint comes
        within the margin of its limits.
    """
    limits = {joint.name: joint.limit for joint in model.upper_leg_joints}
    # Crouching postures have hip angle q1 and knee angle -2 q1 in the left
    # leg, mirrored in the right leg
    max_hip_angle = (
        min(
            limits["left_hip"].upper,
            -limits["right_hip"].lower,
            -0.5 * limits["left_knee"].lower,
            0.5 * limits["right_knee"].upper,
        )
        - margin
    )
    ell = model.leg_segment_length
    return 2.0 * ell * (1.0 - np.cos(max_hip_angle))


def main(robot: str):
    """Crouch up and down in a sine wave while balancing in place.

    Parameters
    ----------
    robot : str
        Name of the robot to simulate, "Upkie" or "Cookie".
    """
    with gym.make(
        f"{robot}-PyBullet-BaseVelocity",
        frequency=200.0,
        frequency_checks=False,
        gui=True,
        nb_substeps=5,
    ) as env:
        env.get_wrapper_attr("set_leg_gain_scale")(LEG_GAIN_SCALES[robot])
        set_crouching_height = env.get_wrapper_attr("set_crouching_height")
        min_height = 0.0  # straight legs
        max_height = compute_max_crouching_height(
            env.unwrapped.model, JOINT_LIMIT_MARGINS[robot]
        )
        velocity = COMPENSATION_VELOCITIES[robot]
        print(
            f"\nCrouching {robot} between {min_height:.3f} m and "
            f"{max_height:.3f} m every {PERIOD} s.\n\n"
            "Crouching brings the knees, and with them the center of mass, "
            "forward of\nthe wheels. The balancer then leans back and drifts "
            "forward to stay\nbalanced. We command a constant ground velocity "
            f"of {velocity} m/s to roughly\ncompensate for this drift.\n\n"
            "Press Ctrl-C to stop..."
        )

        dt = env.unwrapped.dt
        env.reset()
        action = np.array([velocity, 0.0], dtype=np.float32)
        step = 0
        while True:
            # Start from straight legs, crouch to the lowest height at half
            # period, then stand up again
            phase = 2.0 * np.pi * step * dt / PERIOD
            height = min_height + 0.5 * (max_height - min_height) * (
                1.0 - np.cos(phase)
            )
            set_crouching_height(height)
            _, _, terminated, truncated, info = env.step(action)
            if terminated or truncated:
                print("The robot fell, stopping")
                return
            step += 1
            if step % round(1.0 / dt) == 0:  # every second
                spine_observation = info["spine_observation"]
                pitch = spine_observation["base_orientation"]["pitch"]
                ground = spine_observation["wheel_odometry"]["position"]
                print(
                    f"t = {step * dt:4.1f} s: crouching height = "
                    f"{height:.3f} m, base pitch = {pitch:+.3f} rad, "
                    f"ground = {ground:+.2f} m"
                )


def pick_upkie_description() -> str:
    """Pick the robot to simulate from command-line arguments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--robot",
        help="Robot to simulate (default: upkie)",
        choices=["upkie", "cookie"],
        default="upkie",
    )
    return parser.parse_args().robot.capitalize()


if __name__ == "__main__":
    try:
        main(pick_upkie_description())
    except KeyboardInterrupt:
        print("Terminating in response to keyboard interrupt")
