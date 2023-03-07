# PPO balancer

## Training

Training forks its own simulation process, so we can just run:

```
bazel run //agents/ppo_balancer:train
```

## Testing

Start a Bullet spine:

```
bazel run //spines:bullet -c opt -- --nb-substeps 5 --show
```

Then, run a trained policy from the [policies](policies/) folder, for instance let's test `woofers`:

```
bazel run //agents/ppo_balancer:test -- woofers
```
