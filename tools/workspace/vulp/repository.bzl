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
        commit = "16a783afa134fcd9b12b2707bb5a5572919b29f8",
        shallow_since = "1684238238 +0200",
    )
