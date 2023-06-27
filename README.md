# Upkie wheeled biped

[![CI](https://github.com/tasts-robots/upkie/actions/workflows/bazel.yml/badge.svg)](https://github.com/tasts-robots/upkie/actions/workflows/bazel.yml)
[![Making a new robot](https://img.shields.io/badge/making-a%20new%20robot-brightgreen?logo=read-the-docs&style=flat)](https://github.com/tasts-robots/upkie/wiki)
[![Coverage](https://coveralls.io/repos/github/tasts-robots/upkie/badge.svg?branch=main)](https://coveralls.io/github/tasts-robots/upkie?branch=main)
[![Vulp](https://img.shields.io/badge/%F0%9F%A6%8A%20vulp-1.2.0-orange)](https://github.com/tasts-robots/vulp)
[![Chat](https://img.shields.io/matrix/tasts-robots:matrix.org?color=4EB899)](https://app.element.io/#/room/#tasts-robots:matrix.org)

Main repository to build and control **Upkie** wheeled bipeds. Made for Linux 🐧

Questions about building and using an Upkie, or balancing robots in general, are all welcome in the [Discussions](https://github.com/tasts-robots/upkie/discussions) forum or on the [Chat](https://app.element.io/#/room/#tasts-robots:matrix.org).

## Quick sim

If you have Python and a C++ compiler (Ubuntu: ``sudo apt install curl g++ python3-dev``), you can run an Upkie simulation right from the command line:

<img src="https://user-images.githubusercontent.com/1189580/170496331-e1293dd3-b50c-40ee-9c2e-f75f3096ebd8.png" height="100" align="right" />

```console
git clone https://github.com/tasts-robots/upkie.git
cd upkie
./start_wheel_balancer.sh
```

Connect a USB controller to move the robot around 🎮

## Getting started

The code of Upkie is organized into *spines*, which communicate with the simulation or mjbots actuators, and *agents*, the programs that implement robot behaviors (check out [this introduction](https://github.com/tasts-robots/vulp#readme) for more details on agents and spines). In the example above we ran an agent called "wheel balancer". We could also start the simulation spine independently, and let it run waiting for agents to connect:

```console
./start_simulation.sh
```

Let's assume we did that. Now there are two ways we can run an agent: the [PyPI](#pypi) way and the [Bazel](#bazel). Go for PyPI to use the robot in Python, and for Bazel to recompile things from source.

### PyPI

The PyPI distribution is the recommended way to control Upkie in Python. It is already [fast enough](https://github.com/tasts-robots/vulp#performance) for real-time control.

#### Installation

[![PyPI version](https://img.shields.io/pypi/v/upkie)](https://pypi.org/project/upkie/)
[![PyPI downloads](https://pepy.tech/badge/upkie/month)](https://pepy.tech/project/upkie)

```console
pip install upkie
```

#### Example

With a simulation or robot spine running, you can run the following code in a Python interpreter or as a standalone Python script:

```python
import gym
import upkie.envs

upkie.envs.register()

with gym.make("UpkieWheelsEnv-v2", frequency=200.0) as env:
    observation = env.reset()
    action = 0.0 * env.action_space.sample()
    for step in range(1_000_000):
        observation, reward, done, _ = env.step(action)
        if done:
            observation = env.reset()
        pitch = observation[0]
        action[0] = 10.0 * pitch
```

This code will connect to the simulation, reset it and execute the policy continuously. If instead of a simulation spine you are [running a pi3hat spine](#upload-to-the-raspberry-pi) on the robot, this code will control the robot directly.

Check out the ``examples/`` directory for other examples.

### Bazel

We use [Bazel](https://bazel.build/) to build C++ spines that can run on both your host computer and the Raspberry Pi. Everything you see in the ``agents/`` and ``spines/`` directories is built with Bazel, including for instance the wheel and PPO balancers. Bazel makes sure that all versions of all dependencies are correct, which is better for sharing code with other Upkie's (no need to worry about what each user did or did not ``pip install`). Overall, the recommended workflow is to use PyPI packages for prototyping then switch to Bazel when a new agent is ready.

#### Upload to the Raspberry Pi

To run an agent on the Raspberry Pi, we first build it locally and upload it to the Raspberry Pi:

```console
make build
make upload ROBOT=your_upkie
```

Next, connect to the robot and run a pi3hat spine:

```console
$ ssh user@your_upkie
user@your_upkie:~$ cd upkie
user@your_upkie:upkie$ make run_pi3hat_spine
```

Finally, run the agent in a separate shell on the robot:

```
user@robot:upkie$ make run_wheel_balancer
```

## Code overview

### Agents

<dl>
  <dt>Wheel balancer</dt>
  <dd>A baseline agent designed to check out Upkie's physical capabilities. The robot balances with its wheels only, following PD feedback from the head pitch and wheel odometry to wheel velocities, plus a feedforward <a href="https://github.com/tasts-robots/upkie/blob/662d76180e03a855e8810d60eeb5b229c95b68fb/agents/wheel_balancer/wheel_balancer.py#L378-L400">non-minimum phase trick</a> for smoother transitions from standing to rolling.</dd>

  <dt>Pink balancer</dt>
  <dd>A more capable agent that combines wheeled balancing with inverse kinematics computed by <a href="https://github.com/tasts-robots/pink">Pink</a>. This is the controller that runs in the <a href="https://www.youtube.com/shorts/8b36XcCgh7s">first</a> <a href="https://www.youtube.com/watch?v=NO_TkHGS0wQ">two</a> videos of Upkie.</dd>

  <dt>PPO balancer</dt>
  <dd>An agent trained by reinforcement learning to balance with straight legs. Training uses the <code><a href="https://tasts-robots.org/doc/upkie/classenvs_1_1upkie__wheels__env_1_1UpkieWheelsEnv.html#details">UpkieWheelsEnv</a></code> gym environment and the PPO implementation from <a href="https://github.com/DLR-RM/stable-baselines3/">Stable Baselines3</a>.</dd>
</dl>

### Environments

<dl>
  <dt><code><a href="https://tasts-robots.org/doc/upkie/classenvs_1_1upkie__servos__env_1_1UpkieServosEnv.html#details">UpkieServosEnv</a></code></dt>
  <dd>Upkie with full observation and joint position-velocity-torque actions.</dd>
  <dt><code><a href="https://tasts-robots.org/doc/upkie/classenvs_1_1upkie__wheels__env_1_1UpkieWheelsEnv.html#details">UpkieWheelsEnv</a></code></dt>
  <dd>Upkie with full observation but only wheel velocity actions.</dd>
</dl>

Environments are single-threaded rather than vectorized. In return, they run as-is on the real robot.

### Observers

<img src="https://tasts-robots.org/doc/upkie/observers.png" align="right">

<dl>
  <dt><a href="https://tasts-robots.org/doc/upkie/classupkie__locomotion_1_1observers_1_1FloorContact.html#details">Floor contact</a></dt>
  <dd>Detect contact between the wheels and the floor. The pink and wheel balancers use contact as a reset flag for their integrators, to avoid over-spinning the wheels while the robot is in the air.</dd>

  <dt><a href="https://tasts-robots.org/doc/upkie/classupkie__locomotion_1_1observers_1_1WheelContact.html#details">Wheel contact</a></dt>
  <dd>Detect contact between a given wheel and the floor.</dd>

  <dt><a href="https://tasts-robots.org/doc/upkie/classupkie__locomotion_1_1observers_1_1WheelOdometry.html#details">Wheel odometry</a></dt>
  <dd>Measure the relative motion of the floating base with respect to the floor. Wheel odometry is part of their secondary task (after keeping the head straight), which is to stay around the same spot on the floor.</dd>
</dl>

### Spines

<dl>
  <dt>Bullet</dt>
  <dd>Spawn Upkie in a <a href="http://bulletphysics.org/">Bullet</a> simulation. Resetting this spine moves the robot back to its initial configuration in this world.</dd>
  <dt>pi3hat</dt>
  <dd>Spine is made to be called from a Raspberry Pi with an onboard mjbots <a href="https://mjbots.com/products/mjbots-pi3hat-r4-4b">pi3hat</a>. Servos are stopped when the spine is stopped, and switch to <a href="https://github.com/mjbots/moteus/blob/main/docs/reference.md#theory-of-operation">position mode</a> (which is a position-velocity-torque controller) when the spine idles. Check out the <a href="https://tasts-robots.org/doc/vulp/classvulp_1_1spine_1_1StateMachine.html#details">spine state machine</a> for details.</dd>
</dl>
