# Upkie wheeled biped robot

<img src="https://github.com/upkie/upkie/assets/1189580/2fc5ee4a-81b0-425c-83df-558c7147cc59" align="right" width="200" />

[![CI](https://img.shields.io/github/actions/workflow/status/upkie/upkie/ci.yml?branch=main)](https://github.com/upkie/upkie/actions/workflows/ci.yml)
[![Documentation](https://img.shields.io/github/actions/workflow/status/upkie/upkie/docs.yml?branch=main&label=docs)](https://upkie.github.io/upkie/)
[![Coverage](https://coveralls.io/repos/github/upkie/upkie/badge.svg?branch=main)](https://coveralls.io/github/upkie/upkie?branch=main)
[![Conda version](https://img.shields.io/conda/vn/conda-forge/upkie.svg)](https://anaconda.org/conda-forge/upkie)
[![PyPI version](https://img.shields.io/pypi/v/upkie)](https://pypi.org/project/upkie/)

**Upkie** is a fully open source self-balancing wheeled biped robot. It has wheels for balancing and legs to negotiate uneven terrains. Upkies are designed to be buildable at home using only tools and components ordered online, like mjbots actuators. You can develop in Python or C++, on Linux or macOS, then deploy your software to the robot's Raspberry Pi.

This repository contains all the materials needed to build and animate an Upkie:

- Hardware:
    - [Bill of materials](https://github.com/upkie/upkie/wiki/Bill-of-materials)
    - [Build instructions](https://github.com/upkie/upkie/wiki)
    - [Project log](https://hackaday.io/project/185729-upkie-wheeled-biped-robots)
- Software:
    - [Installation](https://github.com/upkie/upkie#installation)
    - [Getting started](https://github.com/upkie/upkie#getting-started)
    - [Documentation](https://upkie.github.io/upkie/)
- Going further:
    - [More agents](https://github.com/upkie/upkie#agents)
    - [More examples](https://github.com/upkie/upkie/tree/main/examples)

Questions are welcome in the [Chat](https://app.element.io/#/room/#upkie:matrix.org) and [Discussions forum](https://github.com/upkie/upkie/discussions).

## Installation

### From conda-forge

```console
conda install -c conda-forge upkie
```

### From PyPI

```console
pip install upkie
```

## Getting started

Let's start a Bullet simulation spine:

<img src="https://github.com/upkie/upkie/blob/main/docs/images/bullet-spine.png" height="100" align="right" />

```console
./start_simulation.sh
```

Click on the robot in the simulator window to apply external forces. Once the simulation spine is running, we can control the robot using one of its Gymnasium environments, for instance:

```python
import gymnasium as gym
import numpy as np
import upkie.envs

upkie.envs.register()

with gym.make("UpkieGroundVelocity-v3", frequency=200.0) as env:
    observation, _ = env.reset()
    gain = np.array([10.0, 1.0, 0.0, 0.1])
    for step in range(1_000_000):
        action = gain.dot(observation).reshape((1,))
        observation, reward, terminated, truncated, _ = env.step(action)
        if terminated or truncated:
            observation, _ = env.reset()
```

The Python code stays the same whether we run a simulation or on a real Upkie. There are also many environments to pick from, check out the list of [Upkie Gymnasium environments](https://upkie.github.io/upkie/environments.html)

## Agents

This main repository distributes Gymnasium environments and [examples](https://github.com/upkie/upkie/tree/main/examples). Larger Upkie agents, some of them with custom C++ spines, have their own repositories:

- [MPC balancer](https://github.com/upkie/mpc_balancer): balance in place using model predictive control.
- [Pink balancer](https://github.com/upkie/pink_balancer): a more advanced agent that can crouch and stand up while balancing.
- [PPO balancer](https://github.com/upkie/ppo_balancer): balance in place with a policy trained by reinforcement learning.
- [PID balancer](https://github.com/upkie/pid_balancer): legacy agent used to test new Upkies with minimal dependencies.

Head over to the [new\_agent](https://github.com/upkie/new_agent) template to create your own, and feel free to open a PR here to add your agent to the list.

## How can I participate?

Contributions are welcome to both the hardware and software of Upkies! If you are a developer/maker with some robotics experience looking to hack on open source, check out the [contribution guidelines](CONTRIBUTING.md). On the software side, you can also report any bug you encounter in the [issue tracker](https://github.com/upkie/upkie/issues).

## Citation

If you built an Upkie or use parts of this project in your works, please cite the project and its contributors:

```bibtex
@software{upkie,
  title = {{Upkie open source wheeled biped robot}},
  author = {Caron, St\'{e}phane and Perrin-Gilbert, Nicolas and Ledoux, Viviane and G\"{o}kbakan, \"{Umit} Bora and Raverdy, Pierre-Guillaume and Raffin, Antonin and Tordjman--Levavasseur, Valentin},
  url = {https://github.com/upkie/upkie},
  license = {Apache-2.0},
  version = {5.2.0},
  year = {2024}
}
```

## See also

- [Awesome Open Source Robots](https://github.com/stephane-caron/awesome-open-source-robots): Upkies are one among many open-source open-hardware robot initiative: check out the others!
