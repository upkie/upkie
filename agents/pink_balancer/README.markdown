# Pink balancer

A baseline agent more capable than the [PID balancer](@ref pid-balancer) that combines wheeled balancing with inverse kinematics computed with [Pink](https://github.com/stephane-caron/pink). This is the controller that runs in the [first](https://www.youtube.com/shorts/8b36XcCgh7s) [two](https://www.youtube.com/watch?v=NO_TkHGS0wQ) videos of Upkie.

## Simulation

To test this agent in simulation, start a Bullet spine:

```console
bazel run //spines:bullet_spine -c opt -- --show
```

Then run the agent with the ``bullet`` configuration in a separate terminal:

```console
bazel run //agents/pink_balancer -- -c bullet
```
