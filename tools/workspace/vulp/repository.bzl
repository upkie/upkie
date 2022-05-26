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
        commit = "35ccc21e58899033e724ddcb3bfdfacc03a9c394",
        shallow_since = "1653556838 +0200"
    )
