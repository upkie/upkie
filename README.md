# Upkie wheeled biped robot

<img src="https://github.com/upkie/upkie/assets/1189580/2fc5ee4a-81b0-425c-83df-558c7147cc59" align="right" width="200" />

[![CI](https://github.com/upkie/upkie/actions/workflows/ci.yml/badge.svg)](https://github.com/upkie/upkie/actions/workflows/ci.yml)
[![Documentation](https://img.shields.io/github/actions/workflow/status/upkie/upkie/docs.yml?branch=main&label=docs)](https://upkie.github.io/upkie/)
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
    - [Getting started](https://github.com/upkie/upkie#getting-started)
    - [Installation](https://github.com/upkie/upkie#installation)
    - [Example](https://github.com/upkie/upkie#example)
    - [Documentation](https://upkie.github.io/upkie/)
- Going further:
    - [More agents](https://github.com/upkie/upkie#agents)
    - [More examples](https://github.com/upkie/upkie/tree/main/examples)

Questions are welcome in the [Chat](https://app.element.io/#/room/#tasts-robots:matrix.org) and [Discussions forum](https://github.com/upkie/upkie/discussions).

## Getting started

You can play with a balancing Upkie right away from your command line:

<img src="https://user-images.githubusercontent.com/1189580/170496331-e1293dd3-b50c-40ee-9c2e-f75f3096ebd8.png" height="100" align="right" />

```console
git clone https://github.com/upkie/upkie.git
cd upkie
./try_pid_balancer.sh
```

Click on the robot in the simulator window to apply external forces. If you have a gaming controller connected to your computer, steer its joysticks to move the robot around 🎮

## Installation

### From conda-forge

[![Conda version](https://img.shields.io/conda/vn/conda-forge/upkie.svg)](https://anaconda.org/conda-forge/upkie)

```console
conda install -c conda-forge upkie
```

### From PyPI

[![PyPI version](https://img.shields.io/pypi/v/upkie)](https://pypi.org/project/upkie)

```console
pip install upkie
```

## Example

First, let's start a simulation:

```console
./start_simulation.sh
```

Once a spine (simulation or real robot) is running, you can control the robot in Python using one of the [Gymnasium environments](https://upkie.github.io/upkie/environments.html). For example, here is a proportional feedback balancer:

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

Executing the code above will balance the simulated Upkie. To run the agent on the robot, do the same but running the [pi3hat spine](https://upkie.github.io/upkie/spines.html#pi3hat-spine) instead of the simulation.

## Agents

The demo agent simply balances in place by [PID control](https://upkie.github.io/upkie/pid-balancer.html). There are more advanced Upkie agents distributed in their own repositories. Check out the ones you are interested in:

- [MPC balancer](https://github.com/upkie/mpc_balancer): balance in place using model predictive control.
- [Pink balancer](https://github.com/upkie/pink_balancer): an extended PID balancer than can crouch and lift its legs.
- [PPO balancer](https://github.com/upkie/ppo_balancer): balance in place with a policy trained by reinforcement learning.

Head over to the [new\_agent](https://github.com/upkie/new_agent) template to create your own.

## Frequently Asked Questions

- [Is the Python API fast enough for real-time control?](https://github.com/orgs/upkie/discussions/240)
- [Can we step a Gym environment faster than real time?](https://github.com/orgs/upkie/discussions/79)
- [How to pair a Bluetooth controller to the Raspberry Pi?](https://github.com/orgs/upkie/discussions/138)

## Citation

If you built an Upkie or use parts of this project in your works, please cite it as follows:

```bibtex
@software{upkie,
  title = {{Upkie open source wheeled biped robot}},
  author = {Caron, St\'{e}phane and Perrin-Gilbert, Nicolas and Ledoux, Viviane and G\"{o}kbakan, \"{Umit} Bora and Raverdy, Pierre-Guillaume and Raffin, Antonin},
  license = {Apache-2.0},
  url = {https://github.com/upkie/upkie},
  version = {4.0.0},
  year = {2024}
}
```
