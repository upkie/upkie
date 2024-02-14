# -*- python -*-
#
# Copyright 2022 Stéphane Caron
# Copyright 2024 Inria

load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

def vulp_repository():
    """
    Clone repository from GitHub and make its targets available for binding.
    """
    git_repository(
        name = "vulp",
        remote = "https://github.com/upkie/vulp.git",
        commit = "b783d69f1d6b2f228a875dc207323270fd0f5b9b",
        shallow_since = "1707476830 +0100",
    )
