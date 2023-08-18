#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2022 Stéphane Caron
# Copyright 2023 Inria
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
#
# This file incorporates work covered by the following copyright and
# permission notice:
#
#     setup-system.py from github.com:mjbots/quad
#     Copyright 2018-2020 Josh Pieper
#     License: Apache-2.0


"""Set up a Raspberry Pi 4b to run software for an Upkie wheeled biped.

The base operating system should already be installed. This script is intended
to be run as root, like: ``sudo ./setup-raspi.py``.
"""

import datetime
import os
import subprocess
import sys
import time

ORIG_SUFFIX = time.strftime(".orig-%Y%m%d-%H%M%S")

# Log each setup step, in case
# ============================

logging_depth = 0


def log_message(message: str, indent: int = 0) -> None:
    logging_indent = " | " * (logging_depth + indent)
    now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
    print(f"{now}: {logging_indent}{message}")


def log_method(func):
    """Decorator to log function calls."""

    def wrapped_function(*args, **kwargs):
        global logging_depth
        try:
            logging_depth += 1
            args_repr = (
                f'"{args[0]}"' + (", ..." if len(args) > 1 else "")
                if args
                else ""
            )
            log_message(f"{func.__name__}({args_repr})", indent=-1)
            result = func(*args, **kwargs)
            return result
        finally:
            logging_depth -= 1

    return wrapped_function


# Utility functions
# =================


@log_method
def run(*args, **kwargs):
    subprocess.check_call(*args, shell=True, **kwargs)


@log_method
def install_packages():
    run("apt-get install --yes python3-pip")  # for moteus-pi3hat
    run("apt-get install --yes screen vim")  # convenient later on
    run("pip install moteus-pi3hat")
    run("pip install upkie")


@log_method
def configure_cpu_isolation(filename="/boot/cmdline.txt"):
    """Make sure CPU isolation is configured.

    Args:
        filename: Path to the boot cmdline configuration file.
    """
    keyword, value = "isolcpus", "1,2,3"
    file_content = [
        x.strip() for x in open(filename, encoding="utf-8").readlines()
    ]
    assert len(file_content) == 1

    new_item = "{}={}".format(keyword, value)
    items = [x.strip() for x in file_content[0].split(" ")]
    present = [x for x in items if x == keyword or x.startswith(keyword + "=")]
    if len(present) > 0:
        new_items = [
            x
            if not (x == keyword or x.startswith(keyword + "="))
            else new_item
            for x in items
        ]
    else:  # len(present) == 0
        new_items = items + [new_item]

    if new_items == items:
        log_message("configure_cpu_isolation(): Already configured")
        return

    log_message(
        "configure_cpu_isolation(): Adding {}={} to {}".format(
            keyword, value, filename
        )
    )

    with open(filename, "w", encoding="utf-8") as f:
        f.write(" ".join(new_items) + "\n")


if __name__ == "__main__":
    if os.geteuid() != 0:
        args = ["sudo", "-E", sys.executable] + sys.argv + [os.environ]
        os.execlpe("sudo", *args)
    install_packages()
    configure_cpu_isolation()
