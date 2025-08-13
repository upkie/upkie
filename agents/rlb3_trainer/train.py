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
from agents.rlb3_trainer.select_env import ENVIRONMENTS, select_env

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


def parse_env_arguments() -> Tuple[argparse.Namespace, List[str]]:
    """
    Parse environment selection arguments.

    Returns:
        Tuple containing:
        - Parsed arguments namespace with the selected environment
        - List of remaining command-line arguments
    """
    parser = argparse.ArgumentParser(description="Upkie environment selection")
    parser.add_argument(
        "-e",
        "--env",
        choices=ENVIRONMENTS,
        help="Environment to use",
    )

    # Parse only known args to avoid conflicts with other parsers
    args, remaining_args = parser.parse_known_args()
    if args.env is None:
        args.env = select_env()

    return args, remaining_args


if __name__ == "__main__":
    upkie.envs.register()
    upkie.logging.disable_warnings()

    # Read custom command-line arguments
    custom_args, remaining_args = parse_env_arguments()

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
