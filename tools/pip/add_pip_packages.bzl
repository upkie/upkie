# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("@rules_python//python:pip.bzl", "pip_install")

def add_pip_packages():
    """
    Add Python package dependencies.

    This function intended to be loaded and called from a WORKSPACE file. If
    your project depends on @upkie_locomotion, you will need to call this
    function from its WORKSPACE to define all @pip_upkie_locomotion
    dependencies.
    """
    pip_install(
        name = "pip_upkie_locomotion",
        requirements = "//tools/pip:requirements.txt",
    )
