#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

## \namespace upkie.exceptions
## \brief Exceptions raised by the Python library.


class UpkieException(Exception):
    """!
    Base class for exceptions raised by Upkie agents.
    """


class FallDetected(UpkieException):
    """!
    Raised when a fall is detected.
    """


class MissingOptionalDependency(UpkieException):
    """!
    Raised when an optional feature lacks a corresponding optional dependency.
    """


class ModelError(UpkieException):
    """!
    Raised when something is wrong in the robot model.
    """


class UpkieRuntimeError(UpkieException, RuntimeError):
    """!
    Runtime error, for instance an invalid call to a library function.
    """


class PerformanceIssue(UpkieException):
    """!
    Exception raised when a performance issue is detected.
    """


class SpineError(UpkieException):
    """!
    Exception raised when the spine sets an error flag in the request field of
    the shared memory map.
    """
