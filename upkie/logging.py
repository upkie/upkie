#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

## \namespace upkie.logging
## \brief Custom logging formatter and logger for the library.

import logging
import time


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
        CRITICAL_RED: str = "\033[41m"
        GREEN: str = "\033[32m"
        RESET: str = "\033[0m"

        self.level_format: dict = {
            logging.CRITICAL: f"[{CRITICAL_RED}{BOLD_WHITE}critical{RESET}]",
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
            "[%(name)s] [%(asctime)s] "
            + self.level_format.get(record.levelno, "[???]")
            + " %(message)s (%(filename)s:%(lineno)d)"
        )
        formatter = logging.Formatter(custom_format, datefmt="%H:%M:%S")
        return formatter.format(record)


## Main logger instance for the Upkie library.
logger = logging.getLogger("upkie")
logger.setLevel(logging.INFO)

## Stream handler for console output with custom formatting.
handler = logging.StreamHandler()
handler.setLevel(logging.DEBUG)
handler.setFormatter(SpdlogFormatter())
logger.addHandler(handler)

## Prevent propagation to root logger to avoid duplicate messages
logger.propagate = False


class RateLimitFilter(logging.Filter):
    """!
    Logging filter that rate-limits repeated warnings from the same source.

    Useful for high-frequency loops where the same warning would otherwise
    spam the terminal on every cycle.
    """

    def __init__(self, min_interval: float = 1.0):
        r"""!
        Initialize the filter.

        \param min_interval Minimum seconds between successive emissions of a
            warning from the same (filename, lineno) location.
        """
        super().__init__()
        self._min_interval = min_interval
        self._last_seen: dict = {}

    def filter(self, record: logging.LogRecord) -> bool:
        r"""!
        Allow a record through only if enough time has passed since the last
        emission from the same source line.

        \param record Log record to evaluate.
        \return True if the record should be emitted, False otherwise.
        """
        key = (record.filename, record.lineno)
        now = time.monotonic()
        if (
            now - self._last_seen.get(key, -self._min_interval)
            < self._min_interval
        ):
            return False
        self._last_seen[key] = now
        return True


def disable_warnings() -> None:
    """!
    Disable all warnings from the upkie module.
    """
    logger.setLevel(logging.ERROR)


def rate_limit_repeated_warnings(min_interval: float = 1.0) -> None:
    r"""!
    Rate-limit warnings that repeat from the same source line.

    After calling this, each unique (file, line) warning is printed at most
    once every ``min_interval`` seconds.

    \param min_interval Minimum seconds between successive prints of the same
        warning location (default: 1.0 s).
    """
    logger.addFilter(RateLimitFilter(min_interval))


__all__ = [
    "RateLimitFilter",
    "disable_warnings",
    "logger",
    "rate_limit_repeated_warnings",
]
