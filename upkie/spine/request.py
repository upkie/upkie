#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

from enum import IntEnum


class Request(IntEnum):
    r"""!
    Flag set by the agent to request an operation from the spine.

    Once the spine has processed the request, it resets the flag to \ref kNone
    in shared memory.
    """

    ## @var kNone
    ## Flag set when there is no active request.
    kNone = 0

    ## @var kObservation
    ## Flag set to indicate an observation is ready to read.
    kObservation = 1

    ## @var kAction
    ## Flag set to indicate an action is ready to read.
    kAction = 2

    ## @var kStart
    ## Flag set to start the spine.
    kStart = 3

    ## @var kStop
    ## Flag set to stop the spine.
    kStop = 4

    ## @var kError
    ## Flag set when the last request was invalid.
    kError = 5
