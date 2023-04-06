# Upkie locomotion

[![Build](https://img.shields.io/github/actions/workflow/status/tasts-robots/upkie_locomotion/bazel.yml?branch=main)](https://github.com/tasts-robots/upkie_locomotion/actions/workflows/bazel.yml)
[![Documentation](https://img.shields.io/badge/docs-online-brightgreen?style=flat)](https://tasts-robots.org/doc/upkie_locomotion/)
[![Coverage](https://coveralls.io/repos/github/tasts-robots/upkie_locomotion/badge.svg?branch=main)](https://coveralls.io/github/tasts-robots/upkie_locomotion?branch=main)
[![PyPI version](https://img.shields.io/pypi/v/upkie_locomotion)](https://pypi.org/project/upkie_locomotion/)
[![Vulp](https://img.shields.io/badge/%F0%9F%A6%8A%20vulp-1.0.0-orange)](https://github.com/tasts-robots/vulp)

Locomotion code for the [Upkie](https://hackaday.io/project/185729-upkie-wheeled-biped-robot) wheeled biped.

Test it straight from the command line on Linux, no installation required:

<img src="https://user-images.githubusercontent.com/1189580/170496331-e1293dd3-b50c-40ee-9c2e-f75f3096ebd8.png" height="100" align="right" />

```console
git clone https://github.com/tasts-robots/upkie_locomotion.git
cd upkie_locomotion
./tools/bazelisk run -c opt //agents/blue_balancer:bullet
```

Connect a USB controller to move the robot around. 🎮

## Getting started

There is no dependency to install thanks to [Bazel](https://bazel.build/), which builds everything locally. (Compilation will only take a while the first time.) The syntax is the same to deploy to the Raspberry Pi on Upkie with [`raspunzel`](https://github.com/tasts-robots/raspunzel).

The code is organized into *spines*, which communicate with the simulator or actuators using the [Vulp](https://github.com/tasts-robots/vulp) C++ library, and *agents*, the main programs that implement behaviors in Python. In the example above we ran the blue agent. We could also run the Bullet spine independently:

```console
bazel run -c opt //spines:bullet -- --show
```

The ``-c opt`` argument to Bazel makes sure we compile optimized code, while the ``--show`` argument to the spine displays the Bullet visualization.

## Agents

<dl>
  <dt>Blue balancer</dt>
  <dd>A baseline agent designed to check out Upkie's physical capabilities. It balances the robot using PD feedback from the head's pitch and wheel odometry to wheel velocities, plus a feedforward <a href="https://github.com/tasts-robots/upkie_locomotion/blob/55a331c6a6a165761a85087b7bea35d1403a6cf9/agents/blue_balancer/wheel_balancer.py#L368">non-minimum phase trick</a> for smoother transitions from standing to rolling. An analytical inverse kinematics is plugged in for crouching and standing up (crouching is controlled from D-pad of the USB controller, if one is found).</dd>

  <dt>Pink balancer</dt>
  <dd>Same as the Blue balancer, but inverse kinematics is computed by <a href="https://github.com/tasts-robots/pink">Pink</a> rather than with a model-specific analytical solution. This is the controller that runs in the <a href="https://www.youtube.com/shorts/8b36XcCgh7s">first</a> <a href="https://www.youtube.com/watch?v=NO_TkHGS0wQ">two</a> videos of Upkie.</dd>

  <dt>PPO balancer</dt>
  <dd>An agent trained by reinforcement learning to balance with straight legs. Training uses the <code><a href="https://tasts-robots.org/doc/upkie_locomotion/classenvs_1_1upkie__wheels__env_1_1UpkieWheelsEnv.html#details">UpkieWheelsEnv</a></code> gym environment and the PPO implementation from <a href="https://github.com/DLR-RM/stable-baselines3/">Stable Baselines3</a>.</dd>
</dl>

## Environments

<dl>
  <dt><code><a href="https://tasts-robots.org/doc/upkie_locomotion/classenvs_1_1upkie__servos__env_1_1UpkieServosEnv.html#details">UpkieServosEnv</a></code></dt>
  <dd>Upkie with full observation and joint position-velocity-torque actions.</dd>
  <dt><code><a href="https://tasts-robots.org/doc/upkie_locomotion/classenvs_1_1upkie__wheels__env_1_1UpkieWheelsEnv.html#details">UpkieWheelsEnv</a></code></dt>
  <dd>Upkie with full observation but only wheel velocity actions.</dd>
</dl>

Environments are single-threaded rather than vectorized. In return, they run as-is on the real robot.

## Observers

<img src="https://tasts-robots.org/doc/upkie_locomotion/observers.png" align="right">

<dl>
  <dt><a href="https://tasts-robots.org/doc/upkie_locomotion/classupkie__locomotion_1_1observers_1_1FloorContact.html#details">Floor contact</a></dt>
  <dd>Detect contact between the wheels and the floor. Both Blue and Pink agents use contact as a reset flag for their integrators, to avoid over-spinning the wheels while the robot is in the air.</dd>

  <dt><a href="https://tasts-robots.org/doc/upkie_locomotion/classupkie__locomotion_1_1observers_1_1WheelContact.html#details">Wheel contact</a></dt>
  <dd>Detect contact between a given wheel and the floor.</dd>

  <dt><a href="https://tasts-robots.org/doc/upkie_locomotion/classupkie__locomotion_1_1observers_1_1WheelOdometry.html#details">Wheel odometry</a></dt>
  <dd>Measure the relative motion of the floating base with respect to the floor. Wheel odometry is part of their secondary task (after keeping the head straight), which is to stay around the same spot on the floor.</dd>
</dl>

## Spines

<dl>
  <dt>Bullet</dt>
  <dd>Spawn Upkie in a <a href="http://bulletphysics.org/">Bullet</a> simulation. Resetting this spine moves the robot back to its initial configuration in this world.</dd>
  <dt>pi3hat</dt>
  <dd>Spine is made to be called from a Raspberry Pi with an onboard mjbots <a href="https://mjbots.com/products/mjbots-pi3hat-r4-4b">pi3hat</a>. Servos are stopped when the spine is stopped, and switch to <a href="https://github.com/mjbots/moteus/blob/main/docs/reference.md#theory-of-operation">position mode</a> (which is a position-velocity-torque controller) when the spine idles. Check out the <a href="https://tasts-robots.org/doc/vulp/classvulp_1_1spine_1_1StateMachine.html#details">spine state machine</a> for details.</dd>
</dl>
