#!/bin/sh

./tools/bazel run //agents/ppo_balancer:run $(realpath $1)
