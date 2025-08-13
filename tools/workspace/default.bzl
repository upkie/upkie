# -*- python -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

load("//tools/workspace/bullet:repository.bzl", "bullet_repository")
load("//tools/workspace/mpacklog:repository.bzl", "mpacklog_repository")
load("//tools/workspace/pi3hat:repository.bzl", "pi3hat_repository")
load("//tools/workspace/upkie_description:repository.bzl", "upkie_description_repository")

def add_default_repositories():
    """
    Declares workspace repositories for all dependencies (other than those
    built into Bazel, of course).

    This function intended to be loaded and called from a WORKSPACE file. If
    your project depends on @upkie, you will need to call this function from
    its WORKSPACE.
    """
    bullet_repository()
    mpacklog_repository()
    pi3hat_repository()
    upkie_description_repository()
