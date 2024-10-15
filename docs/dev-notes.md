# Developer notes {#dev-notes}

## From newbie to sourcie

While newcomers will likely run `start_simulation.sh` and import the `upkie` package in Python, as you get acquainted with the robot and develop your own agents (by forking [upkie/new\_agent](https://github.com/upkie/new_agent)), you may want to contribute some features back to the upstream repository. Here is a short guide on compiling from source to do that.

### Build environment setup {#setup-build}

First, setup a build environment following the instructions for your system:

- [Fedora](https://github.com/orgs/upkie/discussions/100)
- [macOs](https://github.com/orgs/upkie/discussions/159)
- [Ubuntu](https://github.com/orgs/upkie/discussions/101)

To upload software to the robot, you will also need to define the `UPKIE_NAME` environment variable. Assuming the hostname of your Upkie is "michel-strogoff", for example, you can add the following to your shell configuration file:

```
export UPKIE_NAME="michel-strogoff"
```

An IP address will also work.

To make sure your build environment works, try to run a [bullet spine](\ref bullet-spine) from source:

```
make run_bullet_spine
```

Once the simulation builds and runs successfully, move onto building the [pi3hat spine](\ref pi3hat-spine) from source, upload it and run it on your robot.

### Using the local Python module

If you make changes to the `upkie` Python module, you will want to make Python import your local version rather than the one installed from conda-forge or PyPI. There are two ways to do so. One of them is to re-install the package locally, using `pip` from the root of (your local copy of) the repository:

```
cd upkie
pip install -U .
```

Another one is to simply link the module from the directory you are working in. For instance, in the examples directory:

```
cd upkie/examples
ln -s ../upkie ./
```

## Inter-process communication

Upkie's software implements an inter-process communication (IPC) protocol between spines and agents. This protocol is suitable for [real-time](https://en.wiktionary.org/wiki/real-time#English) but not high-frequency (> 1000 Hz) performance, which is fine for balancing and locomotion on Upkies. (If you are wondering whether Python is suitable for real-time applications, we were too, until we [tried it out](https://github.com/orgs/upkie/discussions/240).) All design decisions have their pros and cons. Some pros for this design are:

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
- **Reset:** apply runtime configuration to actuation interface and observers. The reset state is not time-critical, *i.e.*, configuration can take time.
- **Idle:** do nothing.
- **Observe:** write observation from the actuation interface.
- **Act:** send action to the actuation interface.
- **Shutdown:** terminal state, exit the control loop.

There are three possible events:

- `begin`: beginning of a control cycle.
- `end`: end of a control cycle.
- `SIGINT`: the process received an interrupt signal.

Guards, indicated between brackets, may involve two variables:

- `req`: the current [Request](\ref upkie::cpp::spine::Request) from the agent.
- `stop_cycles`: the number of stop commands cycled in the current state (only available in "stop" and "shutdown" states).
