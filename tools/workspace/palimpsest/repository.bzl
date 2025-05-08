# -*- python -*-

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def palimpsest_repository(
        version = "2.3.2",
        sha256 = "4395f1d4e1c6d19bbc754bdafc489aff6a406d660c9b62e08cf99ca8a53c2789"):
    """
    Download release archive from GitHub.

    Args:
        version: Version of the library to download.
        sha256: SHA-256 checksum of the downloaded archive.
    """
    http_archive(
        name = "palimpsest",
        url = "https://github.com/stephane-caron/palimpsest/archive/refs/tags/v{}.tar.gz".format(version),
        sha256 = sha256,
        strip_prefix = "palimpsest-{}".format(version),
    )
