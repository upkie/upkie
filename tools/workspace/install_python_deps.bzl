# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("@rules_python//python:pip.bzl", "pip_install")

def install_python_deps(name = "pip"):
    """
    Install Python packages to an external repository (default: @pip).

    Args:
        name (str, optional): Unique name for the created external repository.

    This function intended to be loaded and called from a WORKSPACE. If your
    project depends on @upkie_locomotion, you will need to call this function
    from its WORKSPACE as well.
    """
    pip_install(
        name = name,
        requirements = Label("//tools/workspace/pip:requirements.txt"),
    )
