echo "==="
echo "$(basename $1)"
echo "==="
cp $1/final.zip ./agents/ppo_balancer/policy/params.zip && ./tools/bazel run //agents/ppo_balancer
