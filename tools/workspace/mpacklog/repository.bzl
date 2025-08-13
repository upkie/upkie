# -*- python -*-

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def mpacklog_repository(
        version = "3.2.0",
        sha256 = "d9931db7fff526f6eae1421e0b12eef7e153b43722191d85e79fe85c9438b3ec"):
    """
    Clone repository from GitHub and make its targets available for binding.

    Args:
        version: Version of the library to download.
        sha256: SHA-256 checksum of the downloaded archive.
    """
    http_archive(
        name = "mpacklog",
        url = "https://github.com/stephane-caron/mpacklog.cpp/archive/refs/tags/v{}.tar.gz".format(version),
        sha256 = sha256,
        strip_prefix = "mpacklog.cpp-{}".format(version),
    )
