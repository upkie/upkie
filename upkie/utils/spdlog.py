#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron
# Copyright 2023 Inria

"""!
Import `logging` from this module to use logging from Python standard library
with formatting similar to spdlog.
"""

import logging


class SpdlogFormatter(logging.Formatter):
    """!
    Custom logging formatter visually consistent with spdlog.
    """

    def __init__(self):
        """!
        Initialize log formatter.
        """
        BOLD_RED: str = "\033[31;1m"
        BOLD_WHITE: str = "\033[37;1m"
        BOLD_YELLOW: str = "\033[33;1m"
        GREEN: str = "\033[32m"
        ON_RED: str = "\033[41m"
        RESET: str = "\033[0m"

        self.level_format: dict = {
            logging.CRITICAL: f"[{ON_RED}{BOLD_WHITE}critical{RESET}]",
            logging.DEBUG: "[debug]",
            logging.ERROR: f"[{BOLD_RED}error{RESET}]",
            logging.INFO: f"[{GREEN}info{RESET}]",
            logging.WARNING: f"[{BOLD_YELLOW}warning{RESET}]",
        }

    def format(self, record):
        r"""!
        Format a given record.

        \param record Record to format.
        """
        custom_format = (
            "[%(asctime)s] "
            + self.level_format.get(record.levelno, "[???]")
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
