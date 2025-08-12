#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

"""!
State-machine request identifiers.
"""

from enum import IntEnum


class Request(IntEnum):
    r"""!
    Flag set by the agent to request an operation from the spine.

    Once the spine has processed the request, it resets the flag to \ref kNone
    in shared memory.

    @var kNone
    Flag set when there is no active request.

    @var kAction
    Flag set to indicate an action has been supplied.

    @var kStart
    Flag set to start the spine.

    @var kStop
    Flag set to stop the spine.

    @var kError
    Flag set when the last request was invalid.
    """

    kNone = 0
    kAction = 1
    kStart = 2
    kStop = 3
    kError = 4
