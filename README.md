# Upkie wheeled biped robot

[![CI](https://github.com/tasts-robots/upkie/actions/workflows/bazel.yml/badge.svg)](https://github.com/tasts-robots/upkie/actions/workflows/bazel.yml)
[![Build instructions](https://img.shields.io/badge/build-instructions-brightgreen?logo=read-the-docs&style=flat)](https://github.com/tasts-robots/upkie/wiki)
[![Documentation](https://img.shields.io/badge/docs-online-brightgreen?style=flat)](https://tasts-robots.github.io/upkie/)
[![Coverage](https://coveralls.io/repos/github/tasts-robots/upkie/badge.svg?branch=main)](https://coveralls.io/github/tasts-robots/upkie?branch=main)
[![PyPI version](https://img.shields.io/pypi/v/upkie)](https://pypi.org/project/upkie/)
[![Chat](https://img.shields.io/badge/matrix-chat-%234eb899)](https://app.element.io/#/room/#tasts-robots:matrix.org)

Build instructions and software for **Upkie** wheeled bipeds. Develop on Linux 🐧 or macOS 🍏, deploy to the robot's Raspberry Pi 🍓. Questions are welcome in the [Discussions](https://github.com/tasts-robots/upkie/discussions) forum or on the [Chat](https://app.element.io/#/room/#tasts-robots:matrix.org).

## Quick sim

If you have Python and a C++ compiler (setup one-liners: [Fedora](https://github.com/tasts-robots/upkie/discussions/100), [Ubuntu](https://github.com/tasts-robots/upkie/discussions/101)), you can run an Upkie simulation right from the command line. It won't install anything on your machine, as everything will run locally from the repository:

<img src="https://user-images.githubusercontent.com/1189580/170496331-e1293dd3-b50c-40ee-9c2e-f75f3096ebd8.png" height="100" align="right" />

```console
git clone https://github.com/tasts-robots/upkie.git
cd upkie
./start_wheel_balancer.sh
```

Click on the robot in the simulator window to apply external forces 😉

## Getting started

1. [Build an Upkie](https://github.com/tasts-robots/upkie/wiki) or [run a simulation](https://github.com/tasts-robots/upkie#simulation-spine)
2. [Run an existing agent](https://github.com/tasts-robots/upkie#running-a-python-agent)
3. [Develop your own agent](https://github.com/tasts-robots/upkie#example-of-a-custom-agent)

## Running a spine

Upkie's code is organized into *spines*, which communicate with the simulation or mjbots actuators, and *agents*, the programs that implement robot behaviors. We use [Bazel](https://bazel.build/) to build spines, both for simulation on your development computer or for running on the robot's Raspberry Pi. Bazel does not install anything on your system: it fetches dependencies with specific versions and builds them locally, making sure the code stays consistent over time. Check out [this introduction](https://github.com/tasts-robots/vulp#readme) for more details.

### Simulation spine

In the example above we ran an agent called "wheel balancer". We could also start the simulation spine independently, and let it run waiting for agents to connect:

```console
./start_simulation.sh
```

This script is just an alias for a Bazel ``run`` command:

```console
./tools/bazelisk run -c opt //spines:bullet -- --show
```

The ``-c opt`` flag selects the optimized compilation mode. It is actually the default in this project, we just show it here for example.

### Robot spine

To run a spine on the robot, we first build it locally and upload it to the onboard Raspberry Pi:

```console
make build
make upload UPKIE_NAME=your_upkie
```

Next, log into the Pi and run a pi3hat spine:

```console
$ ssh foo@robot
foo@robot:~$ cd upkie
foo@robot:upkie$ make run_pi3hat_spine
```

Once the spine is running, you can run any agent in a separate shell on the robot, for example the wheel balancer:

```console
foo@robot:upkie$ make run_wheel_balancer
```

## Running a Python agent

We develop Python agents using the ``upkie`` interface distributed on PyPI. This interface is already [fast enough](https://github.com/tasts-robots/vulp#performance) for real-time control. To install it:

```console
pip install upkie
```

The repository ships a number of [agents](#agents) that have been tested on several Upkie's. You will find them in the [`agents/`](https://github.com/tasts-robots/upkie/tree/main/agents) directory. To run an agent, call its main Python script. For instance, to run the PPO balancer:

```console
python agents/ppo_balancer/main.py
```

## Example of a custom agent

You can develop your own agent using the [environments](#environments) distributed in ``upkie.envs``. For example, [run a spine](#running-a-spine) and try executing the following code in a Python interpreter:

```python
import gymnasium as gym
import upkie.envs

upkie.envs.register()

with gym.make("UpkieGroundVelocity-v1", frequency=200.0) as env:
    observation = env.reset()
    action = 0.0 * env.action_space.sample()
    for step in range(1_000_000):
        observation, reward, done, _ = env.step(action)
        if done:
            observation = env.reset()
        pitch = observation[0]
        action[0] = 10.0 * pitch
```

With a simulation spine, this code will reset the robot's state and execute the policy continuously. In a pi3hat spine, this code will control the robot directly. You can check out the [`examples/`](https://github.com/tasts-robots/upkie/tree/main/examples) directory for more examples.
