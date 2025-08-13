#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

ENVIRONMENTS = [
    "Upkie-PyBullet-Pendulum",
    "Upkie-Genesis-Pendulum",
]


def select_env() -> str:
    """
    Interactively prompt user to select an environment.

    Returns:
        Selected environment name
    """
    print("Available environments:")
    for i, env in enumerate(ENVIRONMENTS, 1):
        print(f"{i}. {env}")

    while True:
        N = len(ENVIRONMENTS)
        choice = input(f"Select environment (1-{N}): ").strip()
        try:
            choice_idx = int(choice) - 1
            if 0 <= choice_idx < len(ENVIRONMENTS):
                return ENVIRONMENTS[choice_idx]
            else:
                print(f"Invalid choice. Please enter 1-{N}.")
        except ValueError:
            print(f"Invalid input. Please enter a number 1-{N}.")
