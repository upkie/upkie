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
        commit = "8e1fc42cd3d6760d8c0d7d6c7014028e1c30b5a9",
        shallow_since = "1711634455 +0100",
    )
