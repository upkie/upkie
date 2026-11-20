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
        commit = "488ef000029999c31d24be4800f58b65cf1f5448",
    )
