# -*- python -*-
#
# SPDX-License-Identifier: Apache-2.0

package(default_visibility = ["//visibility:public"])

py_library(
    name = "config",
    srcs = [
        "__init__.py",
    ],
    data = [
        "spine.yaml",
    ],
)

cc_library(
    name = "layout",
    hdrs = [
        "layout.h",
    ],
    deps = [
        "//upkie/cpp/actuation:servo_layout",
    ],
)
