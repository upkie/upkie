# -*- python -*-
#
# Copyright 2022 Stéphane Caron

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
        "@vulp//vulp/actuation:servo_layout",
    ],
)
