# MPC balancer {#mpc-balancer}

[![Documentation](https://img.shields.io/badge/docs-online-brightgreen?logo=read-the-docs&style=flat)](https://upkie.github.io/upkie/namespacempc__balancer.html)

The MPC balancer allows Upkie to stand upright, balancing with its wheels only, by closed-loop model predictive control. It performs better than the [PID balancer](@ref pid-balancer) with significantly less hacks ;-)

## Simulation

To test this agent in simulation, run the Bullet spine:

```console
./tools/bazelisk run //spines:bullet_spine -- --show
```

Then run the MPC balancer:

```console
./tools/bazelisk run //agents/mpc_balancer
```

## Real robot

To run this agent on a real Upkie, you can use the Makefile at the root of the repository:

```console
$ make build
$ make upload
$ ssh your-upkie
user@your-upkie:~$ make run_mpc_balancer
```
