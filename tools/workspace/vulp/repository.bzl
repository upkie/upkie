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
        remote = "https://github.com/ubgk/vulp.git",
        commit = "69a706f7913243d803d77356353589e26982f9fc",
        shallow_since = "1705597050 +0100",
    )
