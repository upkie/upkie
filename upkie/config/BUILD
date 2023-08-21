# -*- python -*-
#
# Copyright 2022 Stéphane Caron

package(default_visibility = ["//visibility:public"])
load("@pip_upkie//:requirements.bzl", "requirement")

py_library(
    name = "config",
    srcs = [
        "__init__.py",
    ],
    data = [
        "spine.yaml",
    ],
    deps = [
        requirement("pyyaml"),
    ],
)
