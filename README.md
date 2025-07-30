# Upkie wheeled biped robot

<img src="https://github.com/upkie/upkie/assets/1189580/2fc5ee4a-81b0-425c-83df-558c7147cc59" align="right" width="250" />

[![CI](https://img.shields.io/github/actions/workflow/status/upkie/upkie/ci.yml?branch=main)](https://github.com/upkie/upkie/actions/workflows/ci.yml)
[![Documentation](https://img.shields.io/github/actions/workflow/status/upkie/upkie/docs.yml?branch=main&label=docs)](https://upkie.github.io/upkie/)
[![Coverage](https://coveralls.io/repos/github/upkie/upkie/badge.svg?branch=main)](https://coveralls.io/github/upkie/upkie?branch=main)
[![Conda version](https://img.shields.io/conda/vn/conda-forge/upkie.svg)](https://anaconda.org/conda-forge/upkie)
[![PyPI version](https://img.shields.io/pypi/v/upkie)](https://pypi.org/project/upkie/)

**Upkie** is an open source wheeled biped robot. It has wheels for balancing and legs to negotiate uneven terrains. Upkies are designed to be buildable with off-the-shelf tools and components, like mjbots actuators. You can develop in Python or C++, on Linux or macOS, then deploy your agent to the robot's Raspberry Pi. Here are some videos of [Upkies in action](https://www.youtube.com/@upkie).

This repository contains all the materials needed to build and control an Upkie: [build instructions](https://github.com/upkie/upkie/wiki), [documentation](https://upkie.github.io/upkie/) and [examples](https://github.com/upkie/upkie/tree/main/examples). Questions are welcome in the [discussions forum](https://github.com/upkie/upkie/discussions) or in the [chat room](https://matrix.to/#/#upkie:matrix.org).

## Getting started

Upkies come with a default *agent* that can balance and roam around. You can try it out in simulation by:

```console
./start_agent.sh
```

Once the agent is running, you can direct your Upkie using a gamepad 🎮

- **Left joystick:** go forward right backward
- **Right joystick:** turn left or right
- **Directional pad:** down to crouch, up to stand up
- **Right button:** (B on an Xbox controller, red circle on a PS4 controller) emergency stop 🚨 all motors will turn off

Click on the robot in the simulator window to apply external forces and see how the robot reacts.

## Creating your own agent

The software for Upkies is packaged into an `upkie` library for you to develop your own agents. You can install it:

- From conda-forge: `conda install -c conda-forge upkie`
- From PyPI: `pip install upkie`

Your agent will interact with another process, called the *spine*, that will be either a simulation or your robot's actuators. Let's start a Bullet simulation spine:

<img src="https://raw.githubusercontent.com/upkie/upkie/refs/heads/main/docs/images/bullet-spine.png" height="100" align="right" />

```console
./start_simulation.sh
```

Now that the simulation spine is running, we can control the robot in Python, for example:

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

There are other examples in the [examples](https://github.com/upkie/upkie/tree/main/examples), for instance:

- Domain randomization: shows how to add domain-randomization wrappers to an Upkie environment.
- Lying genuflection: genuflect while lying on a horizontal floor.
- Neutral configuration: reset smoothly to the neutral (straight legs) configuration.
- PD balancer: balance by proportional-derivative feedback to wheel velocities.
- Torque balancer: balance by proportional control from base pitch to wheel torques.

### Agents

There are other Upkie agents available in their own repositories:

- [MPC balancer](https://github.com/upkie/mpc_balancer): balance in place using model predictive control.
- [Pink balancer](https://github.com/upkie/pink_balancer): a more advanced agent that can crouch and stand up while balancing.
- [PPO balancer](https://github.com/upkie/ppo_balancer): balance in place with a neural-network policy trained by reinforcement learning.
- [PID balancer](https://github.com/stephane-caron/upkie_pid_balancer): a legacy agent that balances by proportional-integral feedback to wheel velocities.

If you make your own agent, feel free to open a PR to link it from here. There is a [new\_agent](https://github.com/upkie/new_agent) template to get started. (Or you can fork this entire repository, for example if you plan to make changes to the `upkie` module rather than using it as a dependency.)

### Contributing

Contributions are welcome to both the hardware and software of Upkies! If you are a developer/maker with some robotics experience looking to hack on open source, check out the [contribution guidelines](CONTRIBUTING.md). On the software side, you can also report any bug you encounter in the [issue tracker](https://github.com/upkie/upkie/issues).

## Citation

If you built an Upkie or use parts of this project in your works, please cite the project and its contributors:

```bibtex
@software{upkie,
  title = {{Upkie open source wheeled biped robot}},
  author = {Caron, St\'{e}phane and Perrin-Gilbert, Nicolas and Ledoux, Viviane and G\"{o}kbakan, \"{Umit} Bora and Raverdy, Pierre-Guillaume and Raffin, Antonin and Tordjman--Levavasseur, Valentin},
  url = {https://github.com/upkie/upkie},
  license = {Apache-2.0},
  version = {8.1.1},
  year = {2025}
}
```

Don't forget to add yourself to the BibTeX above and to `CITATION.cff` if you contribute to this repository.

## See also

- [Awesome Open Source Robots](https://github.com/stephane-caron/awesome-open-source-robots): Upkies are one among many open-source open-hardware robot initiative: check out the others!
- [Open Dynamic Robot Initiative](https://open-dynamic-robot-initiative.github.io/): An open torque-controlled modular robot architecture for legged locomotion research.
