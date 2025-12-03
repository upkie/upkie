# Upkie wheeled biped robots

<img src="https://github.com/upkie/upkie/assets/1189580/2fc5ee4a-81b0-425c-83df-558c7147cc59" align="right" width="250" />

[![CI](https://img.shields.io/github/actions/workflow/status/upkie/upkie/ci.yml?branch=main)](https://github.com/upkie/upkie/actions/workflows/ci.yml)
[![Documentation](https://img.shields.io/github/actions/workflow/status/upkie/upkie/docs.yml?branch=main&label=docs)](https://upkie.github.io/upkie/)
[![Coverage](https://coveralls.io/repos/github/upkie/upkie/badge.svg?branch=main)](https://coveralls.io/github/upkie/upkie?branch=main)
[![Conda version](https://img.shields.io/conda/vn/conda-forge/upkie.svg)](https://anaconda.org/conda-forge/upkie)
[![PyPI version](https://img.shields.io/pypi/v/upkie)](https://pypi.org/project/upkie/)

**Upkies** are open-source wheeled biped robots. They have wheels for balancing and legs to negotiate uneven terrains. Upkies are designed to be buildable with off-the-shelf tools and components, like mjbots actuators. You can develop in Python or C++, on Linux or macOS, then deploy your behaviors to the robot's Raspberry Pi. Here are some instances of [Upkies in action](https://www.youtube.com/@upkie).

This repository contains all the materials needed to build and control an Upkie. Questions are welcome in the [discussions forum](https://github.com/upkie/upkie/discussions) or in the [chat room](https://matrix.to/#/#upkie:matrix.org).

## Building your own

Upkies come with [step by step build instructions](https://github.com/upkie/upkie/wiki).

## Getting started

Upkies come with a model predictive controller that can balance and roam around. You can try it out in simulation by:

```console
./start_mpc_balancer.sh
```

Once the agent is running, you can direct your Upkie using a gamepad 🎮

- **Left joystick:** go forward right backward
- **Right joystick:** turn left or right
- **Directional pad:** down to crouch, up to stand up
- **Right button:** (B on an Xbox controller, red circle on a PS4 controller) emergency stop 🚨 all motors will turn off

Click on the robot in the simulator window to apply external forces and see how the robot reacts.

## Creating your own behaviors

Software for Upkies comes is packaged in an `upkie` Python library. You can install it:

- From conda-forge: `conda install -c conda-forge upkie`
- From PyPI: `pip install upkie`

When running on the real robot, your code will command the robot's actuators via another process called the *spine*. There are also simulation spines for testing before deploying to a robot. Let's start a Bullet simulation spine:

<img src="https://raw.githubusercontent.com/upkie/upkie/refs/heads/main/docs/images/bullet-spine.png" height="100" align="right" />

```console
./start_simulation.sh
```

Now that we have a spine is running, we can control the robot in Python. For example:

```python
import gymnasium as gym
import numpy as np
import upkie.envs

upkie.envs.register()

with gym.make("Upkie-Spine-Pendulum", frequency=200.0) as env:
    observation, _ = env.reset()
    gain = np.array([10.0, 1.0, 0.0, 0.1])
    for step in range(1_000_000):
        action = gain.dot(observation).reshape((1,))
        observation, reward, terminated, truncated, _ = env.step(action)
        if terminated or truncated:
            observation, _ = env.reset()
```

Other Gymnasium environments provide various levels of absraction to control the robot. They are listed in the [Gym environments](https://upkie.github.io/upkie/gym-environments.html) page of the documentation.

## Going further

### Examples

There are smaller standalone examples in the [examples](https://github.com/upkie/upkie/tree/main/examples) directory. For instance:

- Domain randomization: shows how to add domain-randomization wrappers to an Upkie environment.
- Lying genuflection: genuflect while lying on a horizontal floor.
- Model predictive control: a self-contained MPC balancer
- PD balancer: balance by proportional-derivative feedback to wheel velocities.

Some examples have optional dependencies, like those for the Genesis and PyBullet simulators. You can activate a virtual environment and install them as optional dependencies, or use Pixi:

```console
pixi run --environment genesis ./examples/genesis_balancing.py
```

### Tasks

Upkies come with a set of default behaviors that you can executed as Pixi tasks. To get started, make sure you have [installed `pixi`](https://pixi.sh/latest/#installation).

| Name                   | Task                                                     |
|------------------------|----------------------------------------------------------|
| `try-genesis`          | Run the MPC balancer in Genesis                          |
| `try-pybullet`         | Run the MPC balancer in PyBullet                         |
| `try-spine`            | Run the MPC balancer with a running spine / real robot   |

You can execute a task by `pixi run <task-name>`, for instance:

```console
pixi run try-pybullet
```

Tasks are available both on your machine and on your Upkie's Raspberry Pi (Pixi comes pre-installed on the SD card image). They are implemented by [agents](https://github.com/upkie/upkie/tree/main/agents). You can make your own agents by forking this repository or using the [new\_agent](https://github.com/upkie/new_agent) template to get started.

### Contributing

Contributions are welcome to both the hardware and software of Upkies! Check out the [contribution guidelines](CONTRIBUTING.md).

## Citation

If you built an Upkie or use parts of this project in your works, please cite the project and its contributors:

```bibtex
@software{upkie,
  title = {{Upkie open source wheeled biped robot}},
  author = {Caron, St\'{e}phane and Perrin-Gilbert, Nicolas and Ledoux, Viviane and G\"{o}kbakan, \"{U}mit Bora and Raverdy, Pierre-Guillaume and Raffin, Antonin and Tordjman{-}{-}Levavasseur, Valentin},
  url = {https://github.com/upkie/upkie},
  license = {Apache-2.0},
  version = {10.0.0},
  year = {2025}
}
```

Don't forget to add yourself to the BibTeX above and to `CITATION.cff` if you contribute to the project.

## See also

- [Awesome Open Source Robots](https://github.com/stephane-caron/awesome-open-source-robots): Upkies are one among many open-source open-hardware robot initiative: check out the others!
- [mjbots](https://mjbots.com/): The company that manufactures the brushless servos used in Upkies, that are [open source](https://github.com/mjbots/moteus) in firmware, hardware and software.
- [Open Dynamic Robot Initiative](https://open-dynamic-robot-initiative.github.io/): An open torque-controlled modular robot architecture for legged locomotion research.
