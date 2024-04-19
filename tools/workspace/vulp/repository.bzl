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
        commit = "a56da2c16a5c5a54a3201c3382fdbc3e8d9c1c85",
        shallow_since = "1711634455 +0100",
    )
