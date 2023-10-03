# -*- python -*-
#
# Copyright 2022 Stéphane Caron

workspace(name = "upkie")

# Repositories
# ============
#
# Add default repositories from leaf to root of the dependency tree.

load("//tools/workspace:default.bzl", "add_default_repositories")
add_default_repositories()

# @vulp is added by add_default_repositories
load("@vulp//tools/workspace:default.bzl", add_vulp_repositories = "add_default_repositories")
add_vulp_repositories()

# @palimpsest is added by add_vulp_repositories
load("@palimpsest//tools/workspace:default.bzl", add_palimpsest_repositories = "add_default_repositories")
add_palimpsest_repositories()

# @pi3hat is added by add_vulp_repositories
load("@pi3hat//tools/workspace:default.bzl", add_pi3hat_repositories = "add_default_repositories")
add_pi3hat_repositories()

# @rpi_bazel is added by add_pi3hat_repositories
load("@rpi_bazel//tools/workspace:default.bzl", add_rpi_bazel_repositories = "add_default_repositories")
add_rpi_bazel_repositories()

# Python dependencies
# ===================

# Depends on @rules_python which is a @palimpsest repository
load("//tools/workspace/pip_upkie:parse_deps.bzl", "parse_deps")
parse_deps()
load("@pip_upkie//:requirements.bzl", "install_deps")
install_deps()

# Vulp also has Python dependencies
load("@vulp//tools/workspace/pip_vulp:parse_deps.bzl", parse_vulp_deps = "parse_deps")
parse_vulp_deps()
load("@pip_vulp//:requirements.bzl", install_vulp_deps = "install_deps")
install_vulp_deps()
