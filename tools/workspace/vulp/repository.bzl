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
        commit = "f813d1b97e4ad9368fb33c4718b5ef1916948b40",
        shallow_since = "1705597050 +0100",
    )
