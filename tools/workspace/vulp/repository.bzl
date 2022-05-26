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
        commit = "3e598f72b96eee69861085e5dddd4884cbf9f4de",
        shallow_since = "1653555971 +0200"
    )
