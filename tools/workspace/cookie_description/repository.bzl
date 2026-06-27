# -*- python -*-
#
# SPDX-License-Identifier: Apache-2.0

load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

def cookie_description_repository():
    """
    Clone repository from Codeberg and make its targets available for binding.
    """
    git_repository(
        name = "cookie_description",
        remote = "https://codeberg.org/upkie/cookie_description",
        commit = "46f5e1bb724f4ef06c16d6280b907a6e24e8400d"
    )
