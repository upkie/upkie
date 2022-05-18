# -*- python -*-
#
# Copyright 2022 Stéphane Caron
#
# This file incorporates work covered by the following copyright and
# permission notice:
#
#     Copyright 2018 Josh Pieper, jjp@pobox.com.
#     License: Apache-2.0

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def github_archive(
        name,
        repository,
        commit = None,
        sha256 = "0" * 64,
        build_file = None,
        strip_prefix = None,
        local_override = None,
        **kwargs):
    """
    Download a repository from GitHub as a ZIP archive, decompress it and make
    its targets available for binding.

    Args:
        name: rule name, i.e. the one used in `@name//...` labels when
            referring to this archive from BUILD files.
        repository: GitHub repository as `entity/project`.
        commit: git commit hash or tag.
        sha256: SHA-256 checksum of the downloaded archive.
        build_file: file to use as the BUILD file for this repository. When
            omitted, the BUILD file(s) within the archive will be used.
        strip_prefix: directory prefix to strip from the extracted files. The
            default is "<project>-<commit>".
        local_override: if set, then this local repository will be used instead
            of the one from GitHub.
    """
    if local_override:
        native.local_repository(
            name = name,
            path = local_override,
        )
    else:
        http_archive(
            name = name,
            url = "https://github.com/{repo}/archive/{commit}.zip".format(
                    repo=repository, commit=commit),
            strip_prefix = strip_prefix or "{name}-{version}".format(
                    name=repository.rsplit('/', 1)[-1], version=commit),
            build_file = build_file,
            sha256 = sha256,
            **kwargs)
