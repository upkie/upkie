# Developer notes {#dev-notes}

[TOC]

## Attitude reference system {#ars}

The attitude reference system (ARS) frame is an inertial frame of reference used by the IMU filter onboard the pi3hat. It has its x-axis pointing forward, y-axis pointing to the right and z-axis pointing down ([details](https://github.com/mjbots/pi3hat/blob/ab632c82bd501b9fcb6f8200df0551989292b7a1/docs/reference.md#orientation)). This is not the convention we use in the world frame, and the rotation matrix from the ARS frame to the world frame is:

\f$
R_{WA} = \begin{bmatrix}
    1 & 0 & 0 \\
    0 & -1 & 0 \\
    0 & 0 & -1 \\
\end{bmatrix}
\f$

## Development workflow {#dev-workflow}

While newcomers will likely run `start_simulation.sh` and import the `upkie` package in Python, as you get acquainted with the robot and develop your own agents (in your fork of the repository or using the [new\_agent](https://github.com/upkie/new_agent) template), you may want to contribute some features back upstream. Here is a short guide on compiling from source to do that.

### C++ development {#cpp-dev-workflow}

The C++ development workflow consists of Makefile rules. First, setup a build environment following the instructions for your system:

- [Fedora](https://github.com/orgs/upkie/discussions/100)
- [macOs](https://github.com/orgs/upkie/discussions/159)
- [Ubuntu](https://github.com/orgs/upkie/discussions/101)

To upload software to the robot, you will also need to define the `UPKIE_NAME` environment variable. Assuming the hostname of your Upkie is "michel-strogoff", for example, you can add the following to your shell configuration file:

```
export UPKIE_NAME="michel-strogoff"
```

An IP address will also work.

To make sure your build environment works, try to rebuild the [pi3hat spine](\ref pi3hat-spine) from source:

```
make build
```

You can then upload it by `make upload` and check that it runs on the Raspberry Pi of your Upkie:

```console
$ ssh user@upkie
user@upkie:~$ cd upkie
user@upkie:upkie$ make run_pi3hat_spine
```

Once the spine is running, you can run any agent in a separate shell on the robot, for example the PID balancer from the examples directory:

```console
user@upkie:upkie$ python -m agents.mpc_balancer
```

### Python development {#python-dev-workflow}

The Python development workflow is based on [Pixi](https://pixi.sh/). You can list the available tasks with a dry:

```
pixi run
```

For instance:

- Build and open the documentation: `pixi run docs-open`
- Check linting: `pixi run lint`
- Check unit tests: `pixi run -e test-py12 test`
- Open a shell in a fully configured environment: `pixi shell`

## Inter-process communication

Upkie implements an action-observation loop to control robot actuators from a standalone "agent" process. The inter-process communication (IPC) protocol between the agent and the "spine" process that talks to actuators looks like this:

<p align="center">
    <img alt="Action-observation loop" src="action-observation-loop.png" />
</p>

The agent can be a simple Python script with few dependencies. This separation between agent and spine provides a robot/simulation switch to train or test agents in a simulation spine before running them on a real robot.

This protocol is suitable for [real-time](https://en.wiktionary.org/wiki/real-time#English) but not high-frequency (> 1000 Hz) performance, which is fine for balancing and locomotion on Upkies. (If you are wondering whether Python is suitable for real-time applications, we were too, until we [tried it out](https://github.com/orgs/upkie/discussions/240).) All design decisions have their pros and cons. Some pros for this design are:

- Run the same Python code on simulated and real robots
- Interfaces with to the [mjbots pi3hat](https://mjbots.com/products/mjbots-pi3hat-r4-4b) and mjbots actuators
- Interfaces with to the [Bullet](http://bulletphysics.org/) simulator
- Observer pipeline to extend observations
- Soft real-time: spine-agent loop interactions are predictable and repeatable
- Unit tested, and not only with end-to-end tests

Meanwhile, cons against this design include:

- Low frequency: designed for tasks that run in the 1–400 Hz range (like balancing)
- Soft rather than hard real-time guarantee: the code is empirically reliable by a large margin, that's it
- Weakly-typed IPC: typing is used within agents and spines, but the interface between them is only checked at runtime

## Spine state machine {#spine-state-machine}

Spines run a state machine depicted in the following diagram:

\image html state-machine.png
\image latex state-machine.eps

States have the following purposes:

- **Stop:** do nothing, send stop commands to servos.
- **Reset:** apply runtime configuration to the spine's actuation interface and observers.
    - The reset state is not time-critical, *i.e.*, configuration can take time.
    - When exiting the reset state, the spine writes observations back to the agent.
- **Idle:** do nothing.
- **Act:** send action to the actuation interface and write observation back to the agent.
- **Shutdown:** terminal state, exit the control loop.

There are three possible events:

- `BEGIN`: beginning of a control cycle.
- `END`: end of a control cycle.
- `SIGINT`: the process received an interrupt signal.

Guards (in blue), *i.e.* conditions required to trigger a transition, may involve two variables:

- `req`: the current [Request](\ref upkie::cpp::spine::Request) from the agent.
- `stop_cycles`: the number of stop commands cycled in the current state (only available in "stop" and "shutdown" states).

Read/write operations from/to the shared memory are indicated in red.

## [Advanced] Design notes

### Single spine binary

We want a single spine binary per interface: Bullet, pi3hat, etc. That binary may offer several observer and controller pipelines, configured via the command line.

- Pros:
    - Combinatorial complexity in distributing, installing and running different spines.
    - Allows `Upkie-Spine-*` Gymnasium environment names
    - Simpler for users, especially newcomers.
- Cons:
    - Combinatorial complexity makes the code branching.

### Source code headers

All C++ source files should start with the license line:

```cpp
// SPDX-License-Identifier: Apache-2.0
```

All Python source files should start with the following four lines:

```py
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
```

Optionally, license lines can be followed by copyright lines corresponding to the various contributions to the file (those are optional as in most legal systems copyright is automatically conferred upon the creation of an original work, without the need for a notice), and author lines to identify the individuals who contributed them.
