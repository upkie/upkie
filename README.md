# Upkie wheeled biped robot

<img src="https://github.com/upkie/upkie/assets/1189580/2fc5ee4a-81b0-425c-83df-558c7147cc59" align="right" width="200" />

[![CI](https://github.com/upkie/upkie/actions/workflows/ci.yml/badge.svg)](https://github.com/upkie/upkie/actions/workflows/bazel.yml)
[![Build instructions](https://img.shields.io/badge/hardware-docs-brightgreen?logo=read-the-docs&style=flat)](https://github.com/upkie/upkie/wiki)
[![Documentation](https://img.shields.io/badge/software-docs-brightgreen?logo=read-the-docs&style=flat)](https://upkie.github.io/upkie/)
[![Coverage](https://coveralls.io/repos/github/upkie/upkie/badge.svg?branch=main)](https://coveralls.io/github/upkie/upkie?branch=main)
[![PyPI version](https://img.shields.io/pypi/v/upkie)](https://pypi.org/project/upkie/)
[![Chat](https://img.shields.io/badge/matrix-chat-%234eb899)](https://app.element.io/#/room/#tasts-robots:matrix.org)

**Upkie** is a fully open source self-balancing wheeled biped robot. It has wheels for balancing and legs to negotiate uneven terrains. Upkies are designed to be buildable at home using only tools and components ordered online, like mjbots actuators. You can develop in Python or C++, on Linux or macOS, then deploy your software to the robot's Raspberry Pi.

This repository contains all the materials needed to build and animate an Upkie:

- Hardware:
    - [Bill of materials](https://github.com/upkie/upkie/wiki/Bill-of-materials)
    - [Build instructions](https://github.com/upkie/upkie/wiki)
    - [Project log](https://hackaday.io/project/185729-upkie-wheeled-biped-robots)
- Software:
    - [Installation](https://github.com/upkie/upkie#installation)
    - [Example](https://github.com/upkie/upkie#example)
    - [Documentation](https://upkie.github.io/upkie/)
    - [Agents](https://github.com/upkie/upkie#agents)
    - [More examples](https://github.com/upkie/upkie/tree/main/examples)

Questions are welcome in the [Chat](https://app.element.io/#/room/#tasts-robots:matrix.org) and [Discussions forum](https://github.com/upkie/upkie/discussions).

## Installation

To code for Upkie in Python, just install:

```console
pip install upkie
```

This Python interface is already [fast enough](https://github.com/upkie/vulp#performance) for real-time control. If later on you want to optimize parts of your code, you can move them to C++ [spines](https://upkie.github.io/upkie/spines.html).

## Demo

You can try a demo right from your command line:

<img src="https://user-images.githubusercontent.com/1189580/170496331-e1293dd3-b50c-40ee-9c2e-f75f3096ebd8.png" height="100" align="right" />

```console
git clone https://github.com/upkie/upkie.git
cd upkie
./try_pid_balancer.sh
```

Click on the robot in the simulator window to apply external forces. If you have a gaming controller connected to your computer, steer its joysticks to move the robot around 🎮

## Example

You can develop your own agent using the Gymnasium environments distributed in ``upkie.envs``. For example, here is a simple proportional-feedback balancer:

```python
import gymnasium as gym
import upkie.envs

upkie.envs.register()

with gym.make("UpkieGroundVelocity-v3", frequency=200.0) as env:
    observation, _ = env.reset()
    action = 0.0 * env.action_space.sample()
    for step in range(1_000_000):
        observation, reward, terminated, truncated, _ = env.step(action)
        if terminated or truncated:
            observation, _ = env.reset()
        pitch = observation[0]
        action[0] = 10.0 * pitch
```

To run this agent, you will first need to start a simulation:

```console
./start_simulation.sh
```

Then execute the Python code above. To run the agent on the robot, do the same but running the [pi3hat spine](https://upkie.github.io/upkie/spines.html#pi3hat-spine) instead of the simulation.

## Agents

The demo agent simply balances in place by [PID control](https://upkie.github.io/upkie/pid-balancer.html). There are more advanced Upkie agents distributed in their own repositories. Check out the ones you are interested in:

- [MPC balancer](https://github.com/upkie/mpc_balancer): balance in place using model predictive control.
- [Pink balancer](https://github.com/upkie/pink_balancer): an extended PID balancer than can crouch and lift its legs.
- [PPO balancer](https://github.com/upkie/ppo_balancer): balance in place with a policy trained by reinforcement learning.

Head over to the [new\_agent](https://github.com/upkie/new_agent) template to create your own.
