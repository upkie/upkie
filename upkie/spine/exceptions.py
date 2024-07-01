#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron
# Copyright 2023 Inria


class VulpException(Exception):
    """!
    Base class for exceptions raised by Vulp.
    """


class PerformanceIssue(VulpException):
    """!
    Exception raised when a performance issue is detected.
    """


class SpineError(VulpException):
    """!
    Exception raised when the spine sets an error flag in the request field of
    the shared memory map.
    """
