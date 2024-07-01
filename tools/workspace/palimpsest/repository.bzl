# -*- python -*-

load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

def palimpsest_repository():
    """
    Clone repository from GitHub and make its targets available for binding.
    """
    git_repository(
        name = "palimpsest",
        remote = "https://github.com/upkie/palimpsest",
        commit = "e4e193467840cdc4cf10301babdf2916a60cde70",
        shallow_since = "1716568528 +0200"
    )
