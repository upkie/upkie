# -*- python -*-

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def palimpsest_repository(
        version = "2.3.0",
        sha256 = "b1b1d29c291aa0af84d69e08f4824f410da24e5775a975cb126b63813e59580a"):
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
