# Environments {#environments}

[TOC]

Upkie has environments compatible with the [Gymnasium API](https://gymnasium.farama.org/). The name of an environment is:

```
<Robot>-<Backend>-<ActObs>
```

where ``<Robot>`` is the robot name, ``<Backend>`` selects a physics simulator or a robot spine, and ``<ActObs>`` defines the action and observation spaces in which to interact with the robot. All combinations are available. See [all environments](#all-environments) below. For instance, `Upkie-PyBullet-Pendulum` simulates an Upkie in PyBullet and exposes it as a wheeled inverted pendulum, while `Upkie-Spine-Pendulum` runs the same agent code on the real robot.

While each environment has its own observation and action spaces, all of them also report full [spine observations](\ref observations) in their `info` dictionaries.

## Robots

The robot sets the model (URDF) used by the backend: its kinematics, inertias and joint limits.

- [Upkie](https://github.com/upkie/upkie_description/): the original wheeled biped, with hip, knee and wheel joints on each leg. Its description is installed with the `upkie` package.
- [Cookie](https://codeberg.org/upkie/cookie_description/): a variant robot with the same six joints. It requires the `cookie_description` package, which you can install with `pip install upkie[cookie]`.

## Backends

The backend is what runs on the other side of the environment: a simulator, a spine process, or a mock for testing. Since environments are backend-agnostic, the same agent code runs on any of them.

- [PyBullet](\ref upkie.envs.backends.pybullet_backend.PyBulletBackend): simulates the robot in [PyBullet](https://pybullet.org/) within the Python process. It can run faster than real time (with `regulate_frequency=False`) and in parallel across processes (*e.g.* with Gymnasium vector environments), which makes it the backend of choice for reinforcement learning. It requires the `pybullet` package. Options include `gui` to toggle the simulator window and `inertia_variation` for domain randomization.
- [Spine](\ref upkie.envs.backends.spine_backend.SpineBackend): connects over shared memory to a separate [spine](\ref spines) process, either the [Bullet spine](\ref bullet-spine) in simulation or the [pi3hat spine](\ref pi3hat-spine) on the real robot. This is how agents are deployed: start the spine first, then run your agent with a `Spine` environment. It runs on a single CPU thread at the spine's frequency.
- [Mock](\ref upkie.envs.backends.mock_backend.MockBackend): executes servo commands perfectly, copying commanded joint positions and velocities into its observations, without simulating physics. It is meant for unit tests and for checking that an agent runs end to end.

## Action and observation spaces

The last part of the name selects how the agent interacts with the robot, from low-level servo commands to high-level velocity commands.

- [Servos](\ref upkie_servos_description): the agent commands each servomotor directly (position, velocity, feedforward torque, gains and maximum torque) and observes each joint's position, velocity, torque, temperature and voltage. This is the most expressive space, *e.g.* for whole-body policies.
- [Gyropod](\ref upkie_gyropod_description): legs are held in a fixed crouching posture and the robot behaves like a wheeled inverted pendulum that turns by differential drive. The agent commands ground and yaw velocities, and observes the position, pitch, yaw and their velocities.
- [Pendulum](\ref upkie_pendulum_description): the same as Gyropod restricted to the sagittal plane, with a 1D action (ground velocity) and a 4D observation (pitch, position and their velocities). This is the simplest space to learn balancing.
- [BaseVelocity](\ref upkie_base_velocity_description): an internal [MPC balancer](\ref upkie.controllers.mpc_balancer.MPCBalancer) keeps the robot upright, while the agent commands its linear and yaw velocities and observes its pose in the horizontal plane. This space is suited for navigation tasks.

## All environments {#all-environments}

| Robot | Action and observation spaces | PyBullet backend | Spine backend | Mock backend |
|-------|-------------------------------|------------------|---------------|--------------|
| Upkie | [Servos](\ref upkie_servos_description) | `Upkie-PyBullet-Servos` | `Upkie-Spine-Servos` | `Upkie-Mock-Servos` |
| Upkie | [Gyropod](\ref upkie_gyropod_description) | `Upkie-PyBullet-Gyropod` | `Upkie-Spine-Gyropod` | `Upkie-Mock-Gyropod` |
| Upkie | [Pendulum](\ref upkie_pendulum_description) | `Upkie-PyBullet-Pendulum` | `Upkie-Spine-Pendulum` | `Upkie-Mock-Pendulum` |
| Upkie | [BaseVelocity](\ref upkie_base_velocity_description) | `Upkie-PyBullet-BaseVelocity` | `Upkie-Spine-BaseVelocity` | `Upkie-Mock-BaseVelocity` |
| Cookie | [Servos](\ref upkie_servos_description) | `Cookie-PyBullet-Servos` | `Cookie-Spine-Servos` | `Cookie-Mock-Servos` |
| Cookie | [Gyropod](\ref upkie_gyropod_description) | `Cookie-PyBullet-Gyropod` | `Cookie-Spine-Gyropod` | `Cookie-Mock-Gyropod` |
| Cookie | [Pendulum](\ref upkie_pendulum_description) | `Cookie-PyBullet-Pendulum` | `Cookie-Spine-Pendulum` | `Cookie-Mock-Pendulum` |
| Cookie | [BaseVelocity](\ref upkie_base_velocity_description) | `Cookie-PyBullet-BaseVelocity` | `Cookie-Spine-BaseVelocity` | `Cookie-Mock-BaseVelocity` |

## Usage

Environments are registered with Gymnasium by calling `upkie.envs.register()`. They can then be created with `gymnasium.make()`, passing environment, wrapper and backend options as keyword arguments:

```python
import gymnasium as gym
import upkie.envs

upkie.envs.register()

with gym.make("Upkie-PyBullet-Pendulum", frequency=200.0, gui=True) as env:
    observation, info = env.reset()
    spine_observation = info["spine_observation"]
```
