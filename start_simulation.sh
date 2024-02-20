#!/bin/bash

SCRIPT=$(realpath "$0")
SCRIPTDIR=$(dirname "${SCRIPT}")

URL_ARCHIVE="https://github.com/upkie/upkie/releases/download"

if [ ! -f docs/Doxyfile ]; then
    echo "Unable to find file 'docs/Doxyfile' for version number";
    exit;
fi
# no v at start of version number here
VERSION=$(awk '/^PROJECT_NUMBER/{print $3}' docs/Doxyfile)

SYSTEM=$(uname -s)
ARCH=$(uname -m)

if [[ "$SYSTEM" == Darwin ]]; then
    echo "macOS system"
    if [[ "$ARCH" == x86_64* ]]; then
        echo "x86-64 architecture"
        SPINE_ARCHIVE="$URL_ARCHIVE"/v"$VERSION"/darwin_x86_bullet_spine.tar.gz
    elif [[ "$ARCH" == i*86 ]]; then
        echo "x86-32 architecture"
        SPINE_ARCHIVE="$URL_ARCHIVE"/v"$VERSION"/darwin_x86_bullet_spine.tar.gz
    elif  [[ "$ARCH" == arm* ]]; then
        echo "ARM architecture"
        SPINE_ARCHIVE="$URL_ARCHIVE"/v"$VERSION"/darwin_arm64_bullet_spine.tar.gz
    else
        echo "Unsupported architecture $ARCH"
    fi
elif  [[ "$SYSTEM" == Linux ]]; then
    echo "Linux system"
    if [[ "$ARCH" == x86_64* ]]; then
        echo "x86-64 architecture"
        SPINE_ARCHIVE="$URL_ARCHIVE"/v"$VERSION"/linux_amd64_bullet_spine.tar.gz
    else
        echo "Unsupported architecture: $ARCH"
    fi
else
    echo "Unsupported system: $SYSTEM"
fi

if [[ -z "$SPINE_ARCHIVE" ]]; then
    echo "Building the simulation spine locally...";
    (cd ${SCRIPTDIR} && ${SCRIPTDIR}/tools/bazelisk run //spines:bullet_spine -- --show)
else
    RETCODE=0
    if [ ! -f cache/bullet_spine ]; then
        echo "Downloading the simulation spine from $SPINE_ARCHIVE..."
        mkdir -p cache

        # check that the full operation works - use pipefail as it works for bash/zsh
        (set -o pipefail;  curl -s -L $SPINE_ARCHIVE | tar -C ./cache/ -zxf - ); RETCODE=$?
    fi

    if [[ $RETCODE -eq 0 ]]; then
        echo "Simulation spine downloaded successfully or already in cache, let's roll!";
        cd cache
        ./bullet_spine --show
        # Return code 0 is from Ctrl-C (normal exit)
        # Return code 1 is from closing the simulation GUI
        if [ $? -ne 0 ] && [ $? -ne 1 ]; then
            echo "Could not execute a binary simulation spine, let's build one locally...";
            (cd ${SCRIPTDIR} && ${SCRIPTDIR}/tools/bazelisk run //spines:bullet_spine -- --show)
        fi
    else
        echo "Could not download a simulation spine, let's build one locally...";
        (cd ${SCRIPTDIR} && ${SCRIPTDIR}/tools/bazelisk run //spines:bullet_spine -- --show)
    fi
fi
