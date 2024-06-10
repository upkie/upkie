# -*- python -*-
#
# SPDX-License-Identifier: Apache-2.0

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
        "//upkie/utils",
    ],
)

cc_library(
    name = "version",
    hdrs = [
        "version.h",
    ],
)
