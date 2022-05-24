#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2022 Stéphane Caron

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
    """
    os.sched_setaffinity(0, {cpu})
