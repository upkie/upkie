# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

def upkie_description_repository():
    """
    Clone repository from GitHub and make its targets available for binding.
    """
    git_repository(
        name = "upkie_description",
        remote = "https://github.com/tasts-robots/upkie_description",
        commit = "bb886d0f453c2d6822d431cfd42385bf06052b42",
        shallow_since = "1687961108 +0200"
    )
