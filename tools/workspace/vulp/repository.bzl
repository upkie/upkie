# -*- python -*-
#
# SPDX-License-Identifier: Apache-2.0

load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

def vulp_repository():
    """
    Clone repository from GitHub and make its targets available for binding.
    """
    git_repository(
        name = "vulp",
        remote = "https://github.com/upkie/vulp.git",
        commit = "689c59f4b06137e69e6f01dbc6460a6fa016c560",
        shallow_since = "1711634455 +0100",
    )
