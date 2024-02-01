#!/bin/bash

SCRIPT=$(realpath "$0")
SCRIPTDIR=$(dirname "${SCRIPT}")

URL_ARCHIVE="https://github.com/pgraverdy/upkie/releases/download"

VERSION=$(awk '/^PROJECT_NUMBER/{print $3}' docs/Doxyfile)
# no v at start of version
VERSION="3.3.1-alpha8"

SYSTEM=$(uname -s)
ARCH=$(uname -m)

if [[ "$SYSTEM" == Darwin ]]; then
    echo "MacOS system"
    if [[ "$ARCH" == x86_64* ]]; then
        echo "X64 Architecture"
        SPINE_ARCHIVE="$URL_ARCHIVE"/v"$VERSION"/darwin_x86_bullet_spine.tar.gz
        DIR_ARCHIVE="darwin_x86_bullet_spine"
    elif [[ "$ARCH" == i*86 ]]; then
        echo "X32 Architecture"
        SPINE_ARCHIVE="$URL_ARCHIVE"/v"$VERSION"/darwin_x86_bullet_spine.tar.gz
        DIR_ARCHIVE="darwin_x86_bullet_spine"
    elif  [[ "$ARCH" == arm* ]]; then
        echo "ARM Architecture"
        SPINE_ARCHIVE="$URL_ARCHIVE"/v"$VERSION"/darwin_arm64_bullet_spine.tar.gz
        DIR_ARCHIVE="darwin_arm64_bullet_spine"
    else
        echo "Unsupported architecture"
    fi
elif  [[ "$ARCH" == Linux ]]; then
    echo "Linux system"
    if [[ "$ARCH" == x86_64* ]]; then
        echo "X64 Architecture"
        SPINE_ARCHIVE="$URL_ARCHIVE"/v"$VERSION"/linux_amd64_bullet_spine.tar.gz
        DIR_ARCHIVE="linux_amd64_bullet_spine"
    elif [[ "$ARCH" == i*86 ]]; then
        echo "X32 Architecture"
        SPINE_ARCHIVE="$URL_ARCHIVE"/v"$VERSION"/linux_i386_bullet_spine.tar.gz
        DIR_ARCHIVE="linux_i386_bullet_spine"
    else
        echo "Unsupported architecture"
    fi
else
    echo "Unsupported System"
fi

echo SPINE_ARCHIVE $SPINE_ARCHIVE


if [[ -z "$SPINE_ARCHIVE" ]]; then
    echo "Build spine and use";
    (cd ${SCRIPTDIR} && ${SCRIPTDIR}/tools/bazelisk run //spines:bullet_spine -- --show)
else
    mkdir tmp-bin
    cd tmp-bin

    # check that the full operation works - use pipefail as it works for bash/zsh
    (set -o pipefail;  curl -s -L $SPINE_ARCHIVE | tar -xf - -C . ); RETCODE=$?

    echo RETCODE $RETCODE

    if [[ $RETCODE -eq 0 ]]; then
        echo "Use downloaded spine";
        cd $DIR_ARCHIVE
        ./bullet_spine --show
        exit;
    else        
        echo "Unable to download, Build spine and use";
        (cd ${SCRIPTDIR} && ${SCRIPTDIR}/tools/bazelisk run //spines:bullet_spine -- --show)
    fi
fi


