#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import sys

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
    Configure a process to run as an agent on the Raspberry Pi.
    """
    if os.geteuid() != 0:
        logging.info("Re-running as root to set the CPU affinity")
        args = ["sudo", "-E", sys.executable] + sys.argv + [os.environ]
        os.execlpe("sudo", *args)
    os.sched_setaffinity(0, {__AGENT_CPUID})
