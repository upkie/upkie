# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

exports_files([
    "CPPLINT.cfg",
    ".clang-format",
])

config_setting(
    name = "pi32_config",
    values = {
        "cpu": "armeabihf",
    }
)

config_setting(
    name = "pi64_config",
    values = {
        "cpu": "aarch64",
    }
)

add_lint_tests()
