#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2022 Stéphane Caron
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

"""
Real-time configuration.
"""

import os


def configure_cpu(cpu: int):
    """
    Set the current thread to run on a given CPU core.

    This function is meant to be used in conjunction with the ``isolcpus``
    kernel parameter. Check out the Raspberry Pi page in the documentation.

    Args:
        cpu: CPU core for this thread (on the Pi, CPUID in {0, 1, 2, 3}).

    Notes:
        Calling this function affects only a single thread. A thread created
        with ``pthread_create`` will then inherit the CPU affinity mask of its
        parent.
    """
    os.sched_setaffinity(0, {cpu})
