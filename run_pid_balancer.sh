#!/bin/bash

SCRIPT=$(realpath "$0")
SCRIPTDIR=$(dirname "${SCRIPT}")

read -p "Did you start the simulation? (y/n) " yn

case $yn in
    [Yy]* ) echo "";;
    * ) echo "You will first need to run ./start_simulation.sh"; exit;;
esac

${SCRIPTDIR}/tools/bazelisk run //pid_balancer -- -c bullet
