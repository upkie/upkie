# -*- python -*-
#
# Copyright 2022 Stéphane Caron
# Copyright 2023-2025 Inria

workspace(name = "upkie")

# Add direct dependencies
load("//tools/workspace:default.bzl", "add_default_repositories")
add_default_repositories()

# Add upstream dependencies up from @mpacklog
load("@mpacklog//tools/workspace:default.bzl", add_mpacklog_repositories = "add_default_repositories")
add_mpacklog_repositories()
load("@palimpsest//tools/workspace:default.bzl", add_palimpsest_repositories = "add_default_repositories")
add_palimpsest_repositories()

# Add upstream dependencies up from @pi3hat
load("@pi3hat//tools/workspace:default.bzl", add_pi3hat_repositories = "add_default_repositories")
add_pi3hat_repositories()
load("@rpi_bazel//tools/workspace:default.bzl", add_rpi_bazel_repositories = "add_default_repositories")
add_rpi_bazel_repositories()
