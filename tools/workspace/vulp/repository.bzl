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
        commit = "9d8b5f921cbc6139ac3b469dbbd7b520cd2ab851",
        shallow_since = "1680639965 +0200",
    )
