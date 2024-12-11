# -*- python -*-

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def mpacklog_repository():
    """
    Clone repository from GitHub and make its targets available for binding.
    """
    http_archive(
        name = "mpacklog",
        sha256 = "389cbd249607f1d0a2bbf6d11cbf0690604966e29a8e75e50160cf0faab068c7",
        strip_prefix = "mpacklog.cpp-3.1.0",
        url = "https://github.com/stephane-caron/mpacklog.cpp/archive/refs/tags/v3.1.0.tar.gz",
    )
