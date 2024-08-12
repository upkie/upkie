#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

import os
import sys

from upkie.exceptions import UpkieRuntimeError

from .spdlog import logging

__AGENT_CPUID: int = 3
__MODEL_PATH: str = "/sys/firmware/devicetree/base/model"
__ON_RASPI: bool = False

if os.path.exists(__MODEL_PATH):
    with open(__MODEL_PATH, "r", encoding="utf-8") as fh:
        line = fh.readline()
        __ON_RASPI = line.startswith("Raspberry")


def on_raspi() -> bool:
    """!
    Check whether we are running on a Raspberry Pi.
    """
    return __ON_RASPI


def configure_agent_process() -> None:
    """!
    Configure process to run as an agent on the Raspberry Pi.

    \note This function assumes we are running an underlying script. It won't
    work from an interpreter.
    """
    if hasattr(sys, "ps1"):
        raise UpkieRuntimeError(
            "Cannot configure agent process from an interpreter"
        )
    if os.geteuid() != 0:
        logging.info("Re-running as root to set the CPU affinity")
        args = ["sudo", "-E", sys.executable] + sys.argv + [os.environ]
        os.execlpe("sudo", *args)
    os.sched_setaffinity(0, {__AGENT_CPUID})
