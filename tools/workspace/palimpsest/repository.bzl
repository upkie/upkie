# -*- python -*-

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def palimpsest_repository(
        version = "2.2.1",
        sha256 = "d998b4e195ef75e558f0477da85ffd1961fb2a5b9ad1bafa1a378b6fa8931505"):
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
