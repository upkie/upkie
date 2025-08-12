# -*- python -*-
#
# SPDX-License-Identifier: Apache-2.0

load("@rules_python//python:defs.bzl", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "backends",
    srcs = [
        "__init__.py",
        "backend.py",
    ],
    deps = [
        "//upkie/config",
        "//upkie/exceptions",
        "//upkie/model",
        "//upkie/spine",
        "//upkie/utils",
    ],
)
