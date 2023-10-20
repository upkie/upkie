# Upkie wheeled biped robot

[![CI](https://github.com/tasts-robots/upkie/actions/workflows/bazel.yml/badge.svg)](https://github.com/tasts-robots/upkie/actions/workflows/bazel.yml)
[![Build instructions](https://img.shields.io/badge/build-instructions-brightgreen?logo=read-the-docs&style=flat)](https://github.com/tasts-robots/upkie/wiki)
[![Documentation](https://img.shields.io/badge/docs-online-brightgreen?style=flat)](https://tasts-robots.github.io/upkie/)
[![Coverage](https://coveralls.io/repos/github/tasts-robots/upkie/badge.svg?branch=main)](https://coveralls.io/github/tasts-robots/upkie?branch=main)
[![PyPI version](https://img.shields.io/pypi/v/upkie)](https://pypi.org/project/upkie/)
[![Chat](https://img.shields.io/badge/matrix-chat-%234eb899)](https://app.element.io/#/room/#tasts-robots:matrix.org)

Build instructions and software for **Upkie** wheeled bipeds. Develop in Python or C++ on Linux or macOS, run on the robot's Raspberry Pi. Questions are welcome in the [Discussions](https://github.com/tasts-robots/upkie/discussions) forum or on the [Chat](https://app.element.io/#/room/#tasts-robots:matrix.org).

## Installation

Everything needed to develop with Upkie in Python lies in a single package:

```console
pip install upkie
```

Yes, it's as simple as that. This Python interface is already [fast enough](https://github.com/tasts-robots/vulp#performance) for real-time control. If later on you want to optimize parts of your code, you can move them to C++ [spines](https://tasts-robots.github.io/upkie/spines.html).

## Simulation

Assuming you have a C++ compiler (setup one-liners: [Fedora](https://github.com/tasts-robots/upkie/discussions/100), [Ubuntu](https://github.com/tasts-robots/upkie/discussions/101)), you can run an Upkie simulation right from the command line. It won't install anything on your machine, everything will run locally from the repository:

<img src="https://user-images.githubusercontent.com/1189580/170496331-e1293dd3-b50c-40ee-9c2e-f75f3096ebd8.png" height="100" align="right" />

```console
git clone https://github.com/tasts-robots/upkie.git
cd upkie
./start_wheel_balancer.sh
```

Click on the robot in the simulator window to apply external forces.

## Example

You can develop your own agent using the Gymnasium environments distributed in ``upkie.envs``. For example, here is a simple proportional-feedback balancer:

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

To test this agent on your computer, run the agent and simulation spine in two separate processes: `python this_agent.py` in one shell, and `./start_simulation.sh` in the other.

To run this agent on the robot, `scp` the script to the Raspberry Pi, start a [pi3hat spine](https://tasts-robots.github.io/upkie/spines.html#pi3hat-spine) and execute the script on the Pi itself.

## To go further

- [Build your own Upkie](https://github.com/tasts-robots/upkie/wiki) 🧰
- [More examples](https://github.com/tasts-robots/upkie/tree/main/examples)
