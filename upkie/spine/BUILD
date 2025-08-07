# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "spine",
    srcs = [
        "__init__.py",
        "request.py",
        "serialize.py",
        "spine_interface.py",
        "wait_for_shared_memory.py",
    ],
    deps = [
        "//upkie:exceptions",
        "//upkie:logging",
    ],
)

add_lint_tests()
