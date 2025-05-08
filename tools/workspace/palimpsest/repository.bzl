# -*- python -*-

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def palimpsest_repository(
        version = "2.3.1",
        sha256 = "bf6dd8258b01a2297133581dce6f769ec39141ab6a1f5533e4aaf946c38eecfd"):
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
