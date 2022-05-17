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
        commit = "7c3c7f277e92e0b5e87e789ca570f72b615a0052",
        shallow_since = "1652692050 +0200"
    )
