# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("@rules_python//python:pip.bzl", "pip_install")

def pip_install_upkie_locomotion():
    """
    Install Python package dependencies to @pip_upkie_locomotion.

    This function intended to be loaded and called from a WORKSPACE. If your
    project depends on @upkie_locomotion, you will need to call this function
    from its WORKSPACE to define all @pip_upkie_locomotion dependencies.
    """
    pip_install(
        name = "pip_upkie_locomotion",
        requirements = Label("//tools/pip:requirements.txt"),
    )
