# PPO balancer

## Training a new policy

```
bazel run //agents/ppo_balancer:train [-- --show]
```

## Running a trained policy

Testing a policy assumes the spine is already up and running. For instance:

```
bazel run //spines:bullet -c opt
```

Pick a trained policy from the [policies](policies/) folder and pass its name as argument, for example for a saved policy `foobar.zip`:

```
bazel run //agents/ppo_balancer:run -- foobar
```
