#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2025 Inria
#
# /// script
# dependencies = ["upkie", "pybullet>=3"]
# ///

"""Count contacts between robot wheels and the environment."""

import gymnasium as gym
import numpy as np

import upkie.envs

upkie.envs.register()


def print_contact_details(backend):
    """Print detailed information about all contact points."""
    contact_points = backend.get_contact_points()
    print("  Looking at contacts in details at this step:")
    for i, contact in enumerate(contact_points):
        print(f"    Contact {i + 1}:")
        print(f"      Robot link index: {contact['link_index_a']}")
        print(f"      Contact body: {contact['body_unique_id_b']}")
        print(f"      Contact position on robot: {contact['position_on_a']}")
        print(f"      Contact normal: {contact['contact_normal_on_b']}")
        print(f"      Contact distance: {contact['contact_distance']:.6f} m")
        print(f"      Normal force: {contact['normal_force']:.2f} N")


def run(env):
    """Run the balancing example while monitoring wheel contacts."""
    observation, _ = env.reset()
    simulator = env.unwrapped.backend

    print("Let's count wheel contacts while balancing:")
    for step in range(1000):
        # Simple balance control
        pitch = observation[0]
        ground_position = observation[1]
        ground_velocity = observation[3]
        v = 10.0 * pitch + 1.0 * ground_position + 0.1 * ground_velocity
        action = np.clip([v], -0.9, 0.9)
        observation, reward, terminated, truncated, info = env.step(action)

        # Count wheel contacts
        left_tire_contacts = simulator.get_contact_points("left_wheel_tire")
        right_tire_contacts = simulator.get_contact_points("right_wheel_tire")
        nb_left_contacts = len(left_tire_contacts)
        nb_right_contacts = len(right_tire_contacts)

        # Print contact information
        if step % 100 == 0:
            print(
                f"Step {step:3}: "
                f"left wheel contacts: {nb_left_contacts} contacts, "
                f"right wheel contacts: {nb_right_contacts} contacts"
            )
            if step == 500:
                print_contact_details(simulator)

        if terminated or truncated:
            observation, _ = env.reset()


if __name__ == "__main__":
    with gym.make("Upkie-PyBullet-Pendulum", frequency=200.0, gui=True) as env:
        run(env)
