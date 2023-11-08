#!/bin/sh

cp $(realpath $1) ./agents/ppo_balancer/policy/params.zip
./tools/bazel run //agents/ppo_balancer:run $(realpath $1) -- --training
