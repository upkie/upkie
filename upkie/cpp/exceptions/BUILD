# -*- python -*-
#
# SPDX-License-Identifier: Apache-2.0

load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "exceptions",
    hdrs = [
        "ControllerError.h",
        "FilterError.h",
        "ObserverError.h",
        "PositionCommandError.h",
        "ServoError.h",
        "TypeError.h",
        "UpkieError.h",
    ],
)

add_lint_tests()
