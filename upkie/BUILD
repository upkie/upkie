# -*- python -*-
#
# Copyright 2023 Inria

package(default_visibility = ["//visibility:public"])

py_library(
    name = "python",
    srcs = [
        "__init__.py",
    ],
    deps = [
        "//upkie/config",
        "//upkie/envs",
        "//upkie/observers/base_pitch",
        "//upkie/utils:python",
    ],
)

cc_library(
    name = "version",
    hdrs = [
        "version.h",
    ],
)
