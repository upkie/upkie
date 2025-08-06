#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2025 Inria

import sys
from pathlib import Path

from rl_zoo3.train import train as rl_zoo3_train

import upkie.envs
import upkie.logging

CLI_SETTINGS = {
    "--env": ["Upkie-PyBullet-Pendulum"],
    "--algo": ["ppo"],
    "--eval-freq": ["-1"],  # disabled
    "--env-kwargs": [
        "disable_env_checker:True",
        "gui:False",
        "regulate_frequency:False",
    ],
    "--tensorboard-log": ["./logs/tensorboard/"],
    "--vec-env": ["subproc"],
    "--progress": [],
}

if __name__ == "__main__":
    upkie.envs.register()
    upkie.logging.disable_warnings()

    for key, args in CLI_SETTINGS.items():
        if key not in sys.argv:
            sys.argv.extend([key] + args)

    # Use local hyperparameters file unless others are specified
    if "-conf" not in sys.argv and "--conf-file" not in sys.argv:
        hyperparams_file = Path(__file__).parent / "hyperparams.yml"
        sys.argv.extend(["--conf-file", str(hyperparams_file)])

    # Call the rl_zoo3 train function which handles all argument parsing
    rl_zoo3_train()
