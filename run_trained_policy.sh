#!/bin/sh

POLICY_DIR=$(realpath $1)

if [ ! -d "${POLICY_DIR}" ]; then
    echo "${POLICY_DIR} is not a directory (or does not exist)."
    exit 1
fi

CONFIG=$(realpath $1)/operative_config.gin
PARAMS=$(realpath $1)/final.zip

if [ ! -f "${CONFIG}" ]; then
    echo "Configuration file ${CONFIG} does not exist."
    exit 1
fi

if [ ! -f "${PARAMS}" ]; then
    echo "Policy parameters ${PARAMS} do not exist."
    exit 1
fi

cp -f ${CONFIG} ./agents/ppo_balancer/policy/operative_config.gin
cp -f ${PARAMS} ./agents/ppo_balancer/policy/params.zip

./tools/bazel run //agents/ppo_balancer:run -- --training
