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

if __name__ == "__main__":
    upkie.envs.register()
    upkie.logging.disable_warnings()

    custom_args = (
        ["--env", "Upkie-PyBullet-Pendulum"],
        ["--algo", "ppo"],
        ["--eval-freq", "-1"],
        ["--env-kwargs", "gui:False", "regulate_frequency:False"],
        ["--tensorboard-log", "./logs/tensorboard/"],
        ["--vec-env", "subproc"],
    )

    for args in custom_args:
        key = args[0]
        if key not in sys.argv:
            sys.argv.extend(args)

    # Use custom hyperparameters file if not specified
    if "--conf" not in sys.argv and "--conf-file" not in sys.argv:
        hyperparams_file = Path(__file__).parent / "hyperparams.yml"
        sys.argv.extend(["--conf-file", str(hyperparams_file)])

    # Call the rl_zoo3 train function which handles all argument parsing
    rl_zoo3_train()
