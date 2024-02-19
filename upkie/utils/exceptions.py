#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria


class UpkieException(Exception):
    """Base class for exceptions raised by Upkie agents."""


class FallDetected(UpkieException):
    """Exception raised when a fall is detected."""


class ModelError(UpkieException):
    """Error related to the robot model."""
