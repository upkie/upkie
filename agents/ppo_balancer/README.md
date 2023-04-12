# PPO balancer

## Training

```
bazel run //agents/ppo_balancer:train [-- --show]
```

## Testing

Testing a policy assumes the spine is already up and running. For instance:

```
bazel run //spines:bullet -c opt
```

Pick a trained policy from the [policies](policies/) folder and pass its name as argument, for example for a saved policy `woofers.zip`:

```
bazel run //agents/ppo_balancer:test -- woofers
```
