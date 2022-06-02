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
        commit = "72f63a03f7de0d1221ebcc4839f436dbb4f186df",
        shallow_since = "1654197821 +0200",
    )
