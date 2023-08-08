# PPO balancer

## Training a new policy

```
./tools/bazelisk run //agents/ppo_balancer:train [-- --show]
```

## Running a trained policy

Testing a policy assumes the spine is already up and running, for instance via ``start_simulation.sh`` on your dev machine, or by starting a pi3hat spine on the robot.

Pick a trained policy from the [policies](policies/) folder and pass its name as argument. For instance, if the saved policy is `foobar.zip`, run:

```
./tools/bazelisk run //agents/ppo_balancer:run -- foobar
```
