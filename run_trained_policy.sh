#!/bin/sh

cp $1 ./agents/ppo_balancer/policy/params.zip && ./tools/bazel run //agents/ppo_balancer
