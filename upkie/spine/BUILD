# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "spine",
    srcs = [
        "__init__.py",
        "exceptions.py",
        "request.py",
        "spine_interface.py",
    ],
)

add_lint_tests()
