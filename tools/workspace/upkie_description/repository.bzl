# -*- python -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

def upkie_description_repository():
    """
    Clone repository from GitHub and make its targets available for binding.
    """
    git_repository(
        name = "upkie_description",
        remote = "https://github.com/upkie/upkie_description",
        commit = "19a91ce69cab6742c613cab104986e3f8a18d6a5",
        shallow_since = "1722418945 +0200"
    )
