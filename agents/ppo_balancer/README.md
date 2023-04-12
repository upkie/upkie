# PPO balancer

For both training and testing you will need to start a spine separately:

```
bazel run //spines:bullet -c opt -- --nb-substeps 5 --show
```

The number of substeps should match the agent and spine frequencies defined in ``settings.gin``.

## Training

```
bazel run //agents/ppo_balancer:train
```

## Testing

Run a trained policy from the [policies](policies/) folder, for instance let's test `woofers`:

```
bazel run //agents/ppo_balancer:test -- woofers
```
