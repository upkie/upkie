#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

## \namespace upkie.utils.raspi
## \brief Functions to work on the onboard Raspberry Pi.

import os
from pathlib import Path

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
    Configure process to run on a dedicated CPU ID for better performance.

    The Linux kernel on Upkies is configured with CPU isolation so that we can
    isolate the spine, CAN and Python processes on different CPU cores of the
    Raspberry Pi. Typically the spine process runs on CPU ID 1, its CAN thread
    on CPU ID 2, and the Python interpreter runs on CPU ID 3. (All other system
    processes will run on CPU ID 0 by default.)
    """
    current_affinity = os.sched_getaffinity(0)
    if __AGENT_CPUID in current_affinity and len(current_affinity) == 1:
        logger.info(
            "The agent process is already running on CPU %d",
            __AGENT_CPUID,
        )
        return

    try:
        os.sched_setaffinity(0, {__AGENT_CPUID})
        logger.info(
            "Configured the agent process to run on CPU %d",
            __AGENT_CPUID,
        )
    except PermissionError:
        logger.warning(
            "Error setting CPU affinity of the agent process to CPU %d. "
            "Consider running with 'taskset -c %d' for better control "
            "performance.",
            __AGENT_CPUID,
            __AGENT_CPUID,
        )
        logger.info(
            "The agent process will be running on CPUs: %s",
            str(sorted(current_affinity)),
        )
