#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

from enum import IntEnum


class Request(IntEnum):

    """!
    Flag set by the agent to request an operation from the spine.

    Once the spine has processed the request, it resets the flag to @ref kNone
    in shared memory.
    """

    kNone = 0  # no active request
    kObservation = 1
    kAction = 2
    kStart = 3
    kStop = 4
    kError = 5  # last request was invalid
