#!/bin/bash

SCRIPT=$(realpath "$0")
SCRIPTDIR=$(dirname "${SCRIPT}")

if ! pgrep -f "bullet_spine" > /dev/null
then
    read -p "Simulation not detected, did you start it? (y/n) " yn
    case $yn in
        [Yy]* ) echo "";;
        * ) echo "You will first need to run ./start_simulation.sh"; exit;;
    esac
fi

${SCRIPTDIR}/tools/bazelisk run //pid_balancer -- -c bullet
