#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron
#
# This file incorporates work covered by the following copyright and permission
# notice:
#
#     Copyright 2012-2016 Robot Locomotion Group @ CSAIL
#     License: BSD-3-Clause (see licenses/LICENSE-drake)

"""Linting with clang-format."""

import glob
import os
import platform
import shutil
import subprocess
import sys


def _is_cxx(filename):
    """Returns True only if filename is reasonable to clang-format."""
    root, ext = os.path.splitext(filename)
    _, penultimate_ext = os.path.splitext(root)

    # Don't clang-format CUDA files.
    if penultimate_ext == "cu":
        return False

    # Per https://bazel.build/versions/master/docs/be/c-cpp.html#cc_library
    return ext in [
        ".c",
        ".cc",
        ".cpp",
        ".cxx",
        ".c++",
        ".C",
        ".h",
        ".hh",
        ".hpp",
        ".hxx",
        ".inc",
    ]


def get_clang_format_path():
    """Get path to clang-format."""
    if platform.system() == "Darwin":
        candidates = glob.glob(
            "/opt/homebrew/*/clang-format/**/bin/clang-format",
            recursive=True,
        ) + ["/usr/local/opt/clang-format/bin/clang-format"]
        if len(candidates) < 1:
            raise RuntimeError("clang-format not found in homebrew paths")
        elif len(candidates) > 1:
            print(
                f"WARNING: multiple versions of clang-format: {candidates},"
                f"selecting {candidates[0]}"
            )
        path = candidates[0]
    else:  # platform.system() == "Linux"
        path = shutil.which("clang-format")
    if os.path.isfile(path):
        return path
    raise RuntimeError(f"clang-format not found at {path}")


def _check_clang_format_idempotence(filename):
    clang_format = get_clang_format_path()
    formatter = subprocess.Popen(
        [clang_format, "-style=file", filename], stdout=subprocess.PIPE
    )
    differ = subprocess.Popen(
        ["/usr/bin/diff", "-u", "-", filename],
        stdin=formatter.stdout,
        stdout=subprocess.PIPE,
    )
    changes = differ.communicate()[0]
    if not changes:
        return 0
    print("ERROR: {} needs clang-format".format(filename))
    print(f"You can fix it by `{clang_format} -style=file -i {filename}`")
    return 1


def main():
    """Checks that clang-format is idempotent.

    Checks are performed on each path specified as a command-line argument. We
    exit with code 1 if any of the paths are invalid or clang-format suggests
    any edits. Otherwise, we exit with code 0.
    """
    total_errors = 0
    for filename in sys.argv[1:]:
        if not _is_cxx(filename):
            print("clang_format_lint.py: Skipping " + filename)
            continue
        print("clang_format_lint.py: Linting " + filename)
        total_errors += _check_clang_format_idempotence(filename)
    if total_errors == 0:
        sys.exit(0)
    else:
        sys.exit(1)


if __name__ == "__main__":
    main()
