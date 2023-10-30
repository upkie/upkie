#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2022 Stéphane Caron
# SPDX-License-Identifier: Apache-2.0

"""
Import ``logging`` from this module to use logging from Python standard library
with formatting similar to spdlog.
"""

import logging
from typing import Any, Dict


class SpdlogFormatter(logging.Formatter):

    """
    Custom logging formatter visually consistent with spdlog.
    """

    BOLD_RED: str = "\033[31;1m"
    BOLD_WHITE: str = "\033[37;1m"
    BOLD_YELLOW: str = "\033[33;1m"
    GREEN: str = "\033[32m"
    ON_RED: str = "\033[41m"
    RESET: str = "\033[0m"

    LEVEL_FORMAT: Dict[Any, str] = {
        logging.CRITICAL: f"[{ON_RED}{BOLD_WHITE}critical{RESET}]",
        logging.DEBUG: "[debug]",
        logging.ERROR: f"[{BOLD_RED}error{RESET}]",
        logging.INFO: f"[{GREEN}info{RESET}]",
        logging.WARNING: f"[{BOLD_YELLOW}warning{RESET}]",
    }

    def format(self, record):
        custom_format = (
            "[%(asctime)s] "
            + self.LEVEL_FORMAT.get(record.levelno, "[???]")
            + " %(message)s (%(filename)s:%(lineno)d)"
        )
        formatter = logging.Formatter(custom_format)
        return formatter.format(record)


logger = logging.getLogger()
logger.setLevel(logging.INFO)
handler = logging.StreamHandler()
handler.setLevel(logging.DEBUG)
handler.setFormatter(SpdlogFormatter())
logger.addHandler(handler)
logging.basicConfig(level=logging.INFO)


__all__ = [
    "logging",
]
