# PID balancer {#pid-balancer}

[![Documentation](https://img.shields.io/badge/docs-online-brightgreen?logo=read-the-docs&style=flat)](https://upkie.github.io/upkie/namespacepid__balancer.html)

The PID balancer is a baseline agent designed to check out Upkie's physical capabilities. The robot balances with its wheels only, following PI feedback from the head pitch and wheel odometry to wheel velocities, plus a feedforward [non-minimum phase trick](https://github.com/upkie/upkie/blob/dff2dfb6b6bc148b45d6a3c4ee9a69d295f6ab5a/pid_balancer/wheel_controller.py#L362-L384) for smoother transitions from standing to rolling. The code is factored into wheel controller, doing the actual balancing, and a leg controller holding the hip and knee joints in place.

## Simulation

To test this agent in simulation, run:

```console
bazel run //pid_balancer:bullet
```

## Real robot

To run this agent on a real Upkie, you can use the Makefile at the root of the repository:

```console
$ make build
$ make upload
$ ssh your-upkie
user@your-upkie:~$ make run_pid_balancer
```
