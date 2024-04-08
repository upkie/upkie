# -*- python -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron
# Copyright 2024 Inria

load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

def vulp_repository():
    """
    Clone repository from GitHub and make its targets available for binding.
    """
    git_repository(
        name = "vulp",
        remote = "https://github.com/marlinlm/vulp.git",
        commit = "b8561e8af63729702725ce36c78d78a900470b09",
        shallow_since = "1711634455 +0100",
    )

