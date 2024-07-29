#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron
# Copyright 2023 Inria
#
# This file incorporates work covered by the following copyright and
# permission notice:
#
#     setup-system.py from github.com:mjbots/quad
#     Copyright 2018-2020 Josh Pieper
#     SPDX-License-Identifier: Apache-2.0


"""Configure CPU isolation on Raspberry Pi OS (Debian "bullseye")."""

import os
import sys


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
            (
                x
                if not (x == keyword or x.startswith(keyword + "="))
                else new_item
            )
            for x in items
        ]
    else:  # len(present) == 0
        new_items = items + [new_item]

    if new_items == items:
        print("configure_cpu_isolation(): Already configured")
        return

    print(
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
    configure_cpu_isolation()
