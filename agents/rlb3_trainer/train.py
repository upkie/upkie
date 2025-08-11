#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2025 Inria

import argparse
import sys
from pathlib import Path
from typing import List, Tuple

from rl_zoo3.train import train as rl_zoo3_train

import upkie.envs
import upkie.logging

CLI_SETTINGS = {
    "--algo": ["ppo"],
    "--env-kwargs": [
        "disable_env_checker:True",
        "gui:False",
        "regulate_frequency:False",
    ],
    "--eval-freq": ["-1"],  # disabled
    "--progress": [],
    "--tensorboard-log": ["./logs/tensorboard/"],
    "--vec-env": ["subproc"],
}


def parse_command_line_arguments() -> Tuple[argparse.Namespace, List[str]]:
    r"""!
    Parse custom arguments and get user input for missing values.

    This function handles the environment selection for Upkie RL training.
    If the --env argument is not provided on the command line, it will
    prompt the user interactively to select between available environments.

    \return Tuple containing:
        - Parsed arguments namespace with the selected environment
        - List of remaining command-line arguments to pass to rl_zoo3
    """
    parser = argparse.ArgumentParser(
        description="Upkie RL Training", add_help=False
    )
    parser.add_argument(
        "-e",
        "--env",
        choices=["Upkie-PyBullet-Pendulum", "Upkie-Genesis-Pendulum"],
        help="Environment to train on",
    )

    # Parse only known args to avoid conflicts with rl_zoo3
    args, remaining_args = parser.parse_known_args()

    # Prompt for environment if not specified
    if args.env is None:
        print("Available environments:")
        print("1. Upkie-PyBullet-Pendulum")
        print("2. Upkie-Genesis-Pendulum")

        while True:
            choice = input("Select environment (1 or 2): ").strip()
            if choice == "1":
                args.env = "Upkie-PyBullet-Pendulum"
                break
            elif choice == "2":
                args.env = "Upkie-Genesis-Pendulum"
                break
            else:
                print("Invalid choice. Please enter 1 or 2.")

    return args, remaining_args


if __name__ == "__main__":
    upkie.envs.register()
    upkie.logging.disable_warnings()

    # Read custom command-line arguments
    custom_args, remaining_args = parse_command_line_arguments()

    # Prepare command-line arguments for rl_zoo3
    sys.argv = [sys.argv[0]] + remaining_args
    sys.argv.extend(["--env", custom_args.env])
    for key, args in CLI_SETTINGS.items():
        if key not in sys.argv:
            sys.argv.extend([key] + args)

    # Use local hyperparameters file, unless anothone is specified
    if "-conf" not in sys.argv and "--conf-file" not in sys.argv:
        hyperparams_file = Path(__file__).parent / "hyperparams.yml"
        sys.argv.extend(["--conf-file", str(hyperparams_file)])

    # Now let RL Baselines3 Zoo handle training
    rl_zoo3_train()
