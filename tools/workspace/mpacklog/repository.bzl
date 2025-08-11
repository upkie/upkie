# -*- python -*-

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def mpacklog_repository(
        version = "3.1.2",
        sha256 = "68de1ec5292f53b821da9b01235bd8385250ee3a21435d531f3e372abd7606ce"):
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
