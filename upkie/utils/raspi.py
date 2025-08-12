#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

## \namespace upkie.utils.raspi
## \brief Functions to work on the onboard Raspberry Pi.

import os
import sys
from pathlib import Path

from upkie.exceptions import UpkieRuntimeError

from ..logging import logger

## CPU ID for agent processes on Raspberry Pi.
__AGENT_CPUID = 3

## Path to device tree model file for Raspberry Pi detection.
__MODEL_PATH = Path("/sys/firmware/devicetree/base/model")

## Boolean flag indicating if running on Raspberry Pi hardware.
__ON_RASPI = False


def _detect_raspi() -> bool:
    r"""!
    Detect if running on Raspberry Pi by reading device tree model.

    \return True if running on Raspberry Pi, False otherwise.
    """
    if __MODEL_PATH.exists():
        with __MODEL_PATH.open("r") as fh:
            line = fh.readline()
            return line.startswith("Raspberry")
    return False


# Initialize the module-level flag
__ON_RASPI = _detect_raspi()


def on_raspi() -> bool:
    r"""!
    Check whether we are running on a Raspberry Pi.
    """
    return __ON_RASPI


def configure_agent_process() -> None:
    r"""!
    Configure process to run as an agent on the Raspberry Pi.

    \note This function assumes we are running an underlying script. It won't
    work from an interpreter.
    """
    if hasattr(sys, "ps1"):
        raise UpkieRuntimeError(
            "Cannot configure agent process from an interpreter"
        )
    if os.geteuid() != 0:
        logger.info("Re-running as root to set the CPU affinity")
        args = ["sudo", "-E", sys.executable] + sys.argv + [os.environ]
        os.execlpe("sudo", *args)
    os.sched_setaffinity(0, {__AGENT_CPUID})
