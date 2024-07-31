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
        commit = "067e7f72d4bdfd02789acd7c2c2fbe411c8b6dd8",
        shallow_since = "1721924098 +0200"
    )
