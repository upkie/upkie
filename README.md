# Upkie wheeled biped robots

<img src="https://github.com/upkie/upkie/assets/1189580/2fc5ee4a-81b0-425c-83df-558c7147cc59" align="right" width="250" />

[![CI](https://img.shields.io/github/actions/workflow/status/upkie/upkie/ci.yml?branch=main)](https://github.com/upkie/upkie/actions/workflows/ci.yml)
[![Documentation](https://img.shields.io/github/actions/workflow/status/upkie/upkie/docs.yml?branch=main&label=docs)](https://upkie.github.io/upkie/)
[![Coverage](https://coveralls.io/repos/github/upkie/upkie/badge.svg?branch=main)](https://coveralls.io/github/upkie/upkie?branch=main)
[![Conda version](https://img.shields.io/conda/vn/conda-forge/upkie.svg)](https://anaconda.org/conda-forge/upkie)
[![PyPI version](https://img.shields.io/pypi/v/upkie)](https://pypi.org/project/upkie/)

**Upkies** are open-source wheeled biped robots featuring wheels for balance and legs to negotiate uneven terrains. They are designed to be buildable with off-the-shelf tools and components, such as mjbots actuators. You can develop in Python or C++, on Linux or macOS, then deploy your behaviors directly to the robot's Raspberry Pi. There are videos showing the robots in action on the [Upkies' YouTube channel](https://www.youtube.com/@upkie).

This repository contains all the materials needed to build and control an Upkie. Questions are welcome in the [discussions forum](https://github.com/upkie/upkie/discussions) or in the [chat room](https://matrix.to/#/#upkie:matrix.org).

## Building your own

Upkies come with [step by step build instructions](https://github.com/upkie/upkie/wiki).

## Getting started

Upkies come out-of-the-box with balancing and locomotion capabilities. You can try them out in simulation using [pixi](https://pixi.prefix.dev/):

<img src="https://raw.githubusercontent.com/upkie/upkie/refs/heads/main/docs/images/bullet-spine.png" height="100" align="right" />

```console
pixi run example-follow-joystick
```

Or using [uv](https://docs.astral.sh/uv/):

```console
uv run examples/follow_joystick.py
```

Once the agent is running, you can direct your Upkie using a gamepad 🎮

- **Left joystick ⬆️ ⬇️:** go forward or backward
- **Left joystick ⬅️  ➡️:** turn left or right
- **Right button:** (B on an Xbox controller, red circle on a PS4 controller) emergency stop 🚨

Click on the robot in the simulator window to apply external forces and see how the robot reacts.

## Creating your own behaviors

Software for Upkies comes is packaged in an `upkie` Python library that you can install from `conda` or `pip`:

```console
pip install upkie
```

This library provides [Gymnasium environments](https://upkie.github.io/upkie/gym-environments.html) to control real and simulation robots with the same code. For instance, the following example balances the robot like a wheeled inverted pendulum in PyBullet:

```python
import gymnasium as gym
import numpy as np
import upkie.envs

upkie.envs.register()

with gym.make("Upkie-PyBullet-Pendulum", frequency=200.0) as env:
    observation, _ = env.reset()
    gain = np.array([10.0, 1.0, 0.0, 0.1])
    for step in range(1_000_000):
        action = gain.dot(observation).reshape((1,))
        observation, reward, terminated, truncated, _ = env.step(action)
        if terminated or truncated:
            observation, _ = env.reset()
```

To switch to the real robot, replace "PyBullet" by "Spine" in the environment name.

## Examples

There are smaller standalone examples in the [examples](https://github.com/upkie/upkie/tree/main/examples) directory. For instance:

- Domain randomization: shows how to add domain-randomization wrappers to an Upkie environment.
- Lying genuflection: genuflect while lying on a horizontal floor.
- Model predictive control: a self-contained MPC balancer
- PD balancer: balance by proportional-derivative feedback to wheel velocities.

## Contributing

Contributions are welcome to both the hardware and software of Upkies! Check out the [contribution guidelines](CONTRIBUTING.md).

## Citation

If you built an Upkie or use parts of this project in your works, please cite the project and its contributors:

```bibtex
@software{upkie,
  title = {{Upkie open source wheeled biped robot}},
  author = {Caron, St\'{e}phane and Perrin-Gilbert, Nicolas and Ledoux, Viviane and G\"{o}kbakan, \"{U}mit Bora and Raverdy, Pierre-Guillaume and Raffin, Antonin and Tordjman{-}{-}Levavasseur, Valentin and Arlaud, Etienne and Duclusaud, Marc},
  url = {https://github.com/upkie/upkie},
  license = {Apache-2.0},
  version = {11.0.0},
  year = {2026}
}
```

Don't forget to add yourself to the BibTeX above and to `CITATION.cff` if you contribute to the project.

## See also

### Reinforcement learning playgrounds for Upkies

<img src="https://github.com/user-attachments/assets/f6293fbc-5c59-4e56-bc7f-0ee930503f11" align="right" height="300px">

- [MjLab Upkie](https://github.com/MarcDcls/mjlab_upkie): GPU-accelerated playground based on MjLab and MuJoCo Warp (requires an Nvidia GPU). Can train whole-body policies such as the one depicted to the right.
- [RLB3 upkie](https://github.com/upkie/rlb3_upkie): new CPU playground to train policies for Upkie-Pendulum environments via RL Baselines3 Zoo
- [PPO balancer](https://github.com/upkie/ppo_balancer): legacy CPU playground to train policies for Upkie-Pendulum environments using Stable-Baselines3 ([video](https://www.youtube.com/shorts/bvWgYso1dzI))

### Open-source robotics

- [Awesome Open Source Robots](https://github.com/stephane-caron/awesome-open-source-robots): Upkies are one among many open-source open-hardware robot initiative: check out the others!
- [mjbots](https://mjbots.com/): The company that manufactures the brushless servos used in Upkies, that are [open source](https://github.com/mjbots/moteus) in firmware, hardware and software.
- [Open Dynamic Robot Initiative](https://open-dynamic-robot-initiative.github.io/): An open torque-controlled modular robot architecture for legged locomotion research.
