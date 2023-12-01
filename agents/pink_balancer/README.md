# Pink balancer

A more capable agent that combines wheeled balancing with inverse kinematics computed by <a href="https://github.com/stephane-caron/pink">Pink</a>. This is the controller that runs in the <a href="https://www.youtube.com/shorts/8b36XcCgh7s">first</a> <a href="https://www.youtube.com/watch?v=NO_TkHGS0wQ">two</a> videos of Upkie.

## Simulation

To test this agent in simulation, start a Bullet spine:

```console
bazel run //spines:bullet_spine -c opt -- --show
```

Then run the agent with the ``bullet`` configuration in a separate terminal:

```console
bazel run //agents/pink_balancer -- -c bullet
```
