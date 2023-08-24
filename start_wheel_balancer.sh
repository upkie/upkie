#!/bin/bash

SCRIPT=$(realpath "$0")
SCRIPTDIR=$(dirname "${SCRIPT}")

(cd ${SCRIPTDIR} && ${SCRIPTDIR}/tools/bazelisk run -c opt //agents/wheel_balancer:bullet -- --show)
