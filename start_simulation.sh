#!/bin/bash
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

SCRIPT=$(realpath "$0")
SCRIPTDIR=$(dirname "${SCRIPT}")

DOWNLOAD_URL="https://github.com/upkie/upkie/releases/download"

if [ ! -f docs/Doxyfile ]; then
    echo "❌ Unable to find file 'docs/Doxyfile' for version number"
    exit
fi
# no v at start of version number here
VERSION=$(awk '/^PROJECT_NUMBER/{print $3}' docs/Doxyfile)

SYSTEM=$(uname -s)
ARCH=$(uname -m)

SPINE_ARGS=()
if [ $# -eq 0 ]; then
    SPINE_ARGS=("--show")
else
    SPINE_ARGS=("$@")
fi

for arg in "$@"; do
    if [ "$arg" == "--build" ]; then
        REBUILD=1
        break
    fi
done

if [[ "$SYSTEM" == Darwin ]]; then
    echo "🍏 macOS operating system"
    if [[ "$ARCH" == x86_64* ]]; then
        echo "⚙️  x86 64-bit CPU architecture"
        SPINE_ARCHIVE="$DOWNLOAD_URL"/v"$VERSION"/darwin_x86_bullet_spine.tar.gz
    elif [[ "$ARCH" == i*86 ]]; then
        echo "⚙️  x86 32-bit CPU architecture"
        SPINE_ARCHIVE="$DOWNLOAD_URL"/v"$VERSION"/darwin_x86_bullet_spine.tar.gz
    elif  [[ "$ARCH" == arm* ]]; then
        echo "⚙️  ARM CPU architecture"
        SPINE_ARCHIVE="$DOWNLOAD_URL"/v"$VERSION"/darwin_arm64_bullet_spine.tar.gz
    else
        echo "❌ Unsupported CPU architecture: $ARCH"
    fi
elif  [[ "$SYSTEM" == Linux ]]; then
    echo "🐧 Linux operating system"
    if [[ "$ARCH" == x86_64* ]]; then
        echo "⚙️  x86 64-bit CPU architecture"
        SPINE_ARCHIVE="$DOWNLOAD_URL"/v"$VERSION"/linux_amd64_bullet_spine.tar.gz
    else
        echo "❌ Unsupported CPU architecture: $ARCH"
    fi
else
    echo "❌ Unsupported operating system: $SYSTEM"
fi

if [[ -n "$SPINE_ARCHIVE" ]] && [[ -z "${REBUILD}" ]]; then
    if [ -f cache/bullet_spine ]; then
        OUTPUT=$(./cache/bullet_spine --version)
        CACHE_RC=$?
        if [ "${CACHE_RC}" -eq 0 ]; then
            CACHE_VERSION=$(echo "${OUTPUT}" | awk '{print $4}')
            if [ "${CACHE_VERSION}" != "${VERSION}" ]; then
                echo "Cached version of the simulation spine (${CACHE_VERSION}) is not ${VERSION}"
                rm -f cache/bullet_spine
            fi
        fi
    fi

    CURL_TAR_RC=0
    if [ ! -f cache/bullet_spine ]; then
        echo "Downloading the simulation spine from $SPINE_ARCHIVE..."
        mkdir -p cache

        # check that the full operation works - use pipefail as it works for bash/zsh
        (set -o pipefail; curl -s -L "$SPINE_ARCHIVE" | tar -C ./cache/ -zxf -)
        CURL_TAR_RC=$?
    fi

    if [[ $CURL_TAR_RC -eq 0 ]]; then
        echo "✅ Simulation spine downloaded to cache, let's roll!"
        cd cache || exit
        OUTPUT=$(./bullet_spine "${SPINE_ARGS[@]}" 2>&1)
        SPINE_RC=$?
        # Return code 0 is from Ctrl-C (normal exit)
        # Return code 1 is from closing the simulation GUI
        if [ $SPINE_RC -eq 1 ]; then
            if echo "$OUTPUT" | grep -q "version.*GLIBC"; then
                echo "⚠️ It seems your GLIBC version is not compatible with the downloaded binary"
                REBUILD=1
            fi
        elif [ $SPINE_RC -ne 0 ] && [ $SPINE_RC -ne 1 ]; then
            echo "Simulation spine exited with code $SPINE_RC"
            echo "If this was unexpected, you can also try \`$0 --build\`"
        fi
    else
        REBUILD=1
    fi
fi

if [[ -n "${REBUILD}" ]]; then
    echo "Building the simulation spine from source..."
    (cd "${SCRIPTDIR}" && "${SCRIPTDIR}"/tools/bazelisk run //spines:bullet_spine -- "${SPINE_ARGS[@]}")
fi
