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
        commit = "b51df4bef4c92e2eb2ca73e609816fc7ceaea492",
        shallow_since = "1680708071 +0200"
    )
