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
import pybullet

import upkie.envs

upkie.envs.register()


def print_contact_details(backend):
    """Print detailed information about all contact points."""
    contact_points = backend.get_contact_points()

    for i, contact in enumerate(contact_points):
        print(f"\nContact {i + 1}:")
        print(f"  Robot link index: {contact['link_index_a']}")
        print(f"  Contact body: {contact['body_unique_id_b']}")
        print(f"  Contact position on robot: {contact['position_on_a']}")
        print(f"  Contact normal: {contact['contact_normal_on_b']}")
        print(f"  Contact distance: {contact['contact_distance']:.6f}")
        print(f"  Normal force: {contact['normal_force']:.2f}")


def run(env):
    """Run the balancing example while monitoring wheel contacts."""
    observation, _ = env.reset()
    backend = env.unwrapped.backend
    ground_plane_id = backend._plane_id
    contact_history = []

    print("\nCounting wheel contacts...")
    print("Ground plane ID:", ground_plane_id)
    print("Using link-specific contact monitoring for wheel tires:")
    print("- Left wheel tire: 'left_wheel_tire'")
    print("- Right wheel tire: 'right_wheel_tire'\n")

    for step in range(1000):
        # Balancing control
        pitch = observation[0]
        ground_position = observation[1]
        ground_velocity = observation[3]
        v = 10.0 * pitch + 1.0 * ground_position + 0.1 * ground_velocity
        action = np.array([v])
        action = np.clip(action, -0.9, 0.9)

        # Step simulation
        observation, reward, terminated, truncated, info = env.step(action)

        # Count wheel contacts

        left_tire_contacts = backend.get_contact_points("left_wheel_tire")
        right_tire_contacts = backend.get_contact_points("right_wheel_tire")
        nb_left_contacts = len(left_tire_contacts)
        nb_right_contacts = len(right_tire_contacts)
        contact_history.append((nb_left_contacts, nb_right_contacts))

        # Print contact information every 100 steps
        if step % 100 == 0:
            print(
                f"Step {step:3}: "
                f"left wheel contacts: {nb_left_contacts} contacts, "
                f"right wheel contacts: {nb_right_contacts} contacts"
            )

            # Print detailed contact information occasionally
            if step == 500:
                print_contact_details(backend)

                # Show what the contact links actually are
                contact_points = backend.get_contact_points()
                if contact_points:
                    print("\nActual contact link names:")
                    seen_links = set()
                    for contact in contact_points:
                        link_idx = contact["link_index_a"]
                        if (
                            link_idx not in seen_links
                            and contact["body_unique_id_b"] == ground_plane_id
                        ):
                            seen_links.add(link_idx)
                            if link_idx >= 0:
                                joint_info = pybullet.getJointInfo(
                                    backend._robot_id, link_idx
                                )
                                link_name = joint_info[12].decode("utf-8")
                                print(f"  Link {link_idx}: {link_name}")
                            else:
                                print(f"  Link {link_idx}: base")

        if terminated or truncated:
            observation, _ = env.reset()


if __name__ == "__main__":
    with gym.make("Upkie-PyBullet-Pendulum", frequency=200.0, gui=True) as env:
        run(env)
