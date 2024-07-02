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
        commit = "6639bf2a68eb95da3fc3af5896ceb9c6eaf09679",
        shallow_since = "1716828947 +0200"
    )
