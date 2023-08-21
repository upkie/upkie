#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2022 Stéphane Caron
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
