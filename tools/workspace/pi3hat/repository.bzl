# -*- python -*-

load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

def pi3hat_repository():
    """
    Clone repository from GitHub and make its targets available for binding.
    """
    git_repository(
        name = "pi3hat",
        remote = "https://github.com/mjbots/pi3hat",
        commit = "c82d6f1fa03d5a40630ff6bc522f69927339a797",
        shallow_since = "1655469414 -0400"
    )
