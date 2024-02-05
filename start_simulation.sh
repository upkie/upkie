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
    echo "MacOS system"
    if [[ "$ARCH" == x86_64* ]]; then
        echo "X64 Architecture"
        SPINE_ARCHIVE="$URL_ARCHIVE"/v"$VERSION"/darwin_x86_bullet_spine.tar.gz
    elif [[ "$ARCH" == i*86 ]]; then
        echo "X32 Architecture"
        SPINE_ARCHIVE="$URL_ARCHIVE"/v"$VERSION"/darwin_x86_bullet_spine.tar.gz
    elif  [[ "$ARCH" == arm* ]]; then
        echo "ARM Architecture"
        SPINE_ARCHIVE="$URL_ARCHIVE"/v"$VERSION"/darwin_arm64_bullet_spine.tar.gz
    else
        echo "Unsupported architecture $ARCH"
    fi
elif  [[ "$SYSTEM" == Linux ]]; then
    echo "Linux system"
    if [[ "$ARCH" == x86_64* ]]; then
        echo "X64 Architecture"
        SPINE_ARCHIVE="$URL_ARCHIVE"/v"$VERSION"/linux_amd64_bullet_spine.tar.gz
    else
        echo "Unsupported architecture: $ARCH"
    fi
else
    echo "Unsupported System $SYSTEM"
fi

if [[ -z "$SPINE_ARCHIVE" ]]; then
    echo "Build spine and use";
    (cd ${SCRIPTDIR} && ${SCRIPTDIR}/tools/bazelisk run //spines:bullet_spine -- --show)
else
    echo "Downloading the simulation spine from $SPINE_ARCHIVE..."
    mkdir tmp-bin
    cd tmp-bin

    # check that the full operation works - use pipefail as it works for bash/zsh
    (set -o pipefail;  curl -s -L $SPINE_ARCHIVE | tar -xf - -C . ); RETCODE=$?


    if [[ $RETCODE -eq 0 ]]; then
        echo "Simulation spine downloaded successfully, let's roll!";
        ./bullet_spine --show
        exit;
    else        
        echo "Unable to download, Build spine and use";
        cd ..
        (cd ${SCRIPTDIR} && ${SCRIPTDIR}/tools/bazelisk run //spines:bullet_spine -- --show)
    fi
fi


