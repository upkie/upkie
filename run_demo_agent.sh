#!/bin/bash

SCRIPT=$(realpath "$0")
SCRIPTDIR=$(dirname "${SCRIPT}")

read -p "Did you start the simulation? (y/n) " yn

case $yn in
    [Yy]* ) echo "";;
    * ) echo "You will first need to run ./start_simulation.sh"; exit;;
esac

echo "There are three agents you can try:"
echo ""
echo "1 → PID Balancer"
echo "2 → MPC Balancer"
echo "3 → PPO Balancer"
echo ""
read -p "Which agent do you want to run? [1-3] " agent

pushd ${SCRIPTDIR} > /dev/null 2>&1
case $agent in
    1) ${SCRIPTDIR}/tools/bazelisk run //agents/pid_balancer -- -c bullet;;
    2) ${SCRIPTDIR}/tools/bazelisk run //agents/mpc_balancer;;
    3) ${SCRIPTDIR}/tools/bazelisk run //agents/ppo_balancer:run;;
    *) echo "Unknown agent number"; exit 1;;
esac
