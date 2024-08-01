# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "utils",
    hdrs = [
        "utils.h",
    ],
    deps = [
        "@eigen",
    ],
)

add_lint_tests()
