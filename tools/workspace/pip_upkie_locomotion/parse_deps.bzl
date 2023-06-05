# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("@rules_python//python:pip.bzl", "pip_parse")

def parse_deps():
    """
    Parse PyPI packages to a @pip_upkie external repository.

    This function intended to be loaded and called from your WORKSPACE.
    """
    pip_parse(
        name = "pip_upkie",
        requirements_lock = Label("//tools/workspace/pip_upkie:requirements_lock.txt"),
    )
