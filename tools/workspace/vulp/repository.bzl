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
        remote = "git@github.com:tasts-robots/vulp",
        commit = "95c68d2952fef684da5d92dc56c693017e44238c",
        shallow_since = "1653556452 +0200"
    )
