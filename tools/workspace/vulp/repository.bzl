# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

def vulp_repository():
    """
    Clone repository from GitHub and make its targets available for binding.
    """
    git_repository(
        name = "vulp",
        remote = "https://github.com/tasts-robots/vulp.git",
        commit = "7bc0e961b4755ac94b7fa4d49e8d0aa9083ad25e",
        shallow_since = "1655842288 +0200",
    )
