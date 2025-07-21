# Upkie wheeled biped robot

<img src="https://github.com/upkie/upkie/assets/1189580/2fc5ee4a-81b0-425c-83df-558c7147cc59" align="right" width="250" />

[![CI](https://img.shields.io/github/actions/workflow/status/upkie/upkie/ci.yml?branch=main)](https://github.com/upkie/upkie/actions/workflows/ci.yml)
[![Documentation](https://img.shields.io/github/actions/workflow/status/upkie/upkie/docs.yml?branch=main&label=docs)](https://upkie.github.io/upkie/)
[![Coverage](https://coveralls.io/repos/github/upkie/upkie/badge.svg?branch=main)](https://coveralls.io/github/upkie/upkie?branch=main)
[![Conda version](https://img.shields.io/conda/vn/conda-forge/upkie.svg)](https://anaconda.org/conda-forge/upkie)
[![PyPI version](https://img.shields.io/pypi/v/upkie)](https://pypi.org/project/upkie/)

**Upkie** is an open source wheeled biped robot. It has wheels for balancing and legs to negotiate uneven terrains. Upkies are designed to be buildable with off-the-shelf tools and components, like mjbots actuators. You can develop in Python or C++, on Linux or macOS, then deploy your agent to the robot's Raspberry Pi. Here are some videos of [Upkies in action](https://www.youtube.com/@upkie).

This repository contains all the materials needed to build and control an Upkie: [build instructions](https://github.com/upkie/upkie/wiki), [documentation](https://upkie.github.io/upkie/) and [examples](https://github.com/upkie/upkie/tree/main/examples). Questions are welcome in the [discussions forum](https://github.com/upkie/upkie/discussions) or in the [chat room](https://matrix.to/#/#upkie:matrix.org).

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

<img src="https://raw.githubusercontent.com/upkie/upkie/refs/heads/main/docs/images/bullet-spine.png" height="100" align="right" />

```console
./start_simulation.sh
```

Click on the robot in the simulator window to apply external forces. Once the simulation spine is running, we can control the robot using one of its Gymnasium environments, for instance:

```python
import gymnasium as gym
import numpy as np
import upkie.envs

upkie.envs.register()

with gym.make("UpkieGroundVelocity-v4", frequency=200.0) as env:
    observation, _ = env.reset()
    gain = np.array([10.0, 1.0, 0.0, 0.1])
    for step in range(1_000_000):
        action = gain.dot(observation).reshape((1,))
        observation, reward, terminated, truncated, _ = env.step(action)
        if terminated or truncated:
            observation, _ = env.reset()
```

The Python code is the same whether we run in simulation or on a real Upkie. Head over to the [examples](https://github.com/upkie/upkie/tree/main/examples) directory for more use cases.

## Gymnasium environments

Upkie has environments compatible with the [Gymnasium API](https://gymnasium.farama.org/), for instance:

- `UpkieGroundVelocity`: keep legs straight and balance with the wheels.
- `UpkieServos`: control joint servos directly with torque feedforward and position-velocity feedback.

Check out the full [list of environments](https://upkie.github.io/upkie/gym-environments.html) for details.

## Agents

Larger Upkie agents have their own repositories:

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
  version = {8.1.0},
  year = {2025}
}
```

## See also

- [Awesome Open Source Robots](https://github.com/stephane-caron/awesome-open-source-robots): Upkies are one among many open-source open-hardware robot initiative: check out the others!
- [Open Dynamic Robot Initiative](https://open-dynamic-robot-initiative.github.io/): An open torque-controlled modular robot architecture for legged locomotion research.
