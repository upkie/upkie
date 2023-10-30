#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria
# SPDX-License-Identifier: Apache-2.0


class UpkieException(Exception):
    """Base class for exceptions raised by Upkie agents."""


class FallDetected(UpkieException):
    """Exception raised when a fall is detected."""
