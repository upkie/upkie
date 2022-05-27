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
        commit = "7d82d4099348f04589b66e512626408f32313182",
        shallow_since = "1653636263 +0200"
    )
