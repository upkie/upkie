# -*- python -*-

load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

def palimpsest_repository():
    """
    Clone repository from GitHub and make its targets available for binding.
    """
    git_repository(
        name = "palimpsest",
        remote = "https://github.com/upkie/palimpsest",
        commit = "edb864af38d9ccef0e158b93dbb544b1000f773f",
        shallow_since = "1723131375 +0200"
    )
