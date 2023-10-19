# Pink balancer

## Simulation

To test this agent in simulation, start a Bullet spine:

```console
bazel run //spines:bullet_spine -c opt -- --show
```

Then run the agent with the ``bullet`` configuration in a separate terminal:

```console
bazel run //agents/pink_balancer -- -c bullet
```
