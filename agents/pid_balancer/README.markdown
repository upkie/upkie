# PID balancer

This agent balances Upkie with its wheels only, by PI feedback from the head pitch and wheel odometry to wheel velocities, plus a feedforward "non-minimum phase trick" (see the comments in `wheel_balancer.py` for details) for smoother transitions from standing to rolling. It consists of a wheel controller, doing the actual balancing, and a leg controller holding the hip and knee joints in place.

To test this agent in simulation, run:

```console
bazel run //agents/pid_balancer:bullet
```
