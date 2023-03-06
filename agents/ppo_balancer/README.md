# PPO balancer

## Installation

This agent depends on [Stable Baselines3](https://github.com/DLR-RM/stable-baselines3) which we should be installed separately [for now](https://github.com/tasts-robots/upkie_locomotion/issues/11):

```
pip install stable-baselines3
```

## Training

Training forks its own simulation process, so we can just run:

```
bazel run //agents/ppo_balancer:train
```

## Testing

Start a Bullet spine:

```
bazel run //spines:bullet -c opt -- --show
```

Then, run a trained policy from the [policies](policies/) folder, for instance let's test `woofers`:

```
bazel run //agents/ppo_balancer:test -- woofers
```
