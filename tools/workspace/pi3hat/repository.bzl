# -*- python -*-

load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

def pi3hat_repository():
    """
    Clone repository from GitHub and make its targets available for binding.
    """
    git_repository(
        name = "pi3hat",
        remote = "https://github.com/mjbots/pi3hat",
        commit = "4a3158c831da125fa9c96d64378515c1fdb2083f",
        shallow_since = "1718203493 -0400"
    )
