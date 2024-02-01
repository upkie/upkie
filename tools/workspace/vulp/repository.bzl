# -*- python -*-
#
# Copyright 2022 Stéphane Caron
# Copyright 2024 Inria

load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

def vulp_repository():
    """
    Clone repository from GitHub and make its targets available for binding.
    """
    git_repository(
        name = "vulp",
        remote = "https://github.com/ubgk/vulp.git",
        commit = "8217d8473204e5c74b28f0e003ea59447eb34ccc",
        shallow_since = "1705597050 +0100",
    )
