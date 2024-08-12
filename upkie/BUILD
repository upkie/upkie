# -*- python -*-
#
# SPDX-License-Identifier: Apache-2.0

package(default_visibility = ["//visibility:public"])

py_library(
    name = "exceptions",
    srcs = ["exceptions.py"],
)

py_library(
    name = "upkie",
    srcs = [
        "__init__.py",
    ],
    deps = [
        "//upkie/config",
        "//upkie/envs",
        "//upkie/spine",
        "//upkie/utils",
        ":exceptions",
    ],
)
