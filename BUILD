# -*- python -*-
#
# Copyright 2022 Stéphane Caron

package(default_visibility = ["//visibility:public"])

exports_files([
    "CPPLINT.cfg",
    ".clang-format",
])

config_setting(
    name = "linux",
    constraint_values = ["@platforms//os:linux"],
)

config_setting(
    name = "osx",
    constraint_values = ["@platforms//os:osx"],
)

config_setting(
    name = "pi64_config",
    values = {
        "cpu": "aarch64",
    }
)

py_library(
    name = "python",
    imports = ["."],
    deps = [
        "//upkie:python",
    ],
)
