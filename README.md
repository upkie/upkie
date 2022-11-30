# Upkie locomotion

![C++ version](https://img.shields.io/badge/C++-17/20-blue.svg?style=flat)
[![Build](https://img.shields.io/github/workflow/status/tasts-robots/upkie_locomotion/build)](https://github.com/tasts-robots/upkie_locomotion/actions/workflows/build.yml)
[![Test](https://img.shields.io/github/workflow/status/tasts-robots/upkie_locomotion/test?label=test)](https://github.com/tasts-robots/upkie_locomotion/actions/workflows/test.yml)
[![Coverage](https://coveralls.io/repos/github/tasts-robots/upkie_locomotion/badge.svg?branch=main)](https://coveralls.io/github/tasts-robots/upkie_locomotion?branch=main)
[![Documentation](https://img.shields.io/badge/docs-online-brightgreen?logo=read-the-docs&style=flat)](https://tasts-robots.org/doc/upkie_locomotion/)
[![Vulp](https://img.shields.io/badge/%F0%9F%A6%8A%20vulp-1.0.0-orange)](https://github.com/tasts-robots/vulp)

Locomotion code for the [Upkie](https://hackaday.io/project/185729-upkie-wheeled-biped-robot) wheeled biped.

### Try it out!

No installation required on Linux:

<img src="https://user-images.githubusercontent.com/1189580/170496331-e1293dd3-b50c-40ee-9c2e-f75f3096ebd8.png" height="100" align="right" />

```console
git clone https://github.com/tasts-robots/upkie_locomotion.git
cd upkie_locomotion
./tools/bazelisk run -c opt //agents/blue_balancer:bullet
```

Connect a USB controller to move the robot around. 🎮

There is no dependency to install on Linux thanks to [Bazel](https://bazel.build/), which builds all dependencies and runs the Python controller in one go. (This will take a while the first time.) The syntax is the same to deploy to the Raspberry Pi on the robot.

## Overview

The code is organized into *spines*, which communicate with the simulator or actuators using the [Vulp](https://github.com/tasts-robots/vulp) C++ library, and *agents*, the main programs that implement behaviors in Python.

* [Agents](#agents)
    * [Blue balancer](#blue-balancer)
    * [Pink balancer](#pink-balancer)
* [Observers](#observers)
    * [Floor contact](#floor-contact)
    * [Wheel contact](#wheel-contact)
    * [Wheel odometry](#wheel-odometry)
* [Spines](#spines)
    * [Bullet](#bullet)
    * [pi3hat](#pi3hat)

### Agents

#### 🔵 Blue balancer

A 200 Hz agent designed to check out Upkie's physical capabilities. It is repeatable, and a good entry point for newcomers. It balances the robot using PID feedback from the head's pitch and wheel odometry to wheel velocities, plus a feedforward [non-minimum phase trick](https://github.com/tasts-robots/upkie_locomotion/blob/55a331c6a6a165761a85087b7bea35d1403a6cf9/agents/blue_balancer/wheel_balancer.py#L368) for smoother transitions from standing to rolling. An analytical inverse kinematics is also plugged in for crouching and standing up. (It is connected to the D-pad of the USB controller if one is found.)

#### 🟣 Pink balancer

Same as the Blue balancer, but inverse kinematics is computed by [Pink](https://github.com/tasts-robots/pink) rather than with a model-specific analytical solution. This is the controller that runs in the [first](https://www.youtube.com/shorts/8b36XcCgh7s) [two](https://www.youtube.com/watch?v=NO_TkHGS0wQ) videos of Upkie.

### Observers

<img src="https://tasts-robots.org/doc/upkie_locomotion/observers.png" align="right">

The following observers are used to detect contacts and keep track of where the wheels are on the ground:

#### [Floor contact](https://tasts-robots.org/doc/upkie_locomotion/classupkie__locomotion_1_1observers_1_1FloorContact.html#details)

Detect contact between the wheels and the floor. Both Blue and Pink agents use contact as a reset flag for their integrators, to avoid over-spinning the wheels while the robot is in the air.

#### [Wheel contact](https://tasts-robots.org/doc/upkie_locomotion/classupkie__locomotion_1_1observers_1_1WheelContact.html#details)

Detect contact between a given wheel and the floor.

#### [Wheel odometry](https://tasts-robots.org/doc/upkie_locomotion/classupkie__locomotion_1_1observers_1_1WheelOdometry.html#details)

Measure the relative motion of the floating base with respect to the floor. Wheel odometry is part of their secondary task (after keeping the head straight), which is to stay around the same spot on the floor.

### Spines

#### 👾 Bullet

Spawn Upkie in a [Bullet](http://bulletphysics.org/) simulation. Resetting this spine moves the robot back to its initial configuration in this world.

#### 🤖 pi3hat

This spine is made to be called from a Raspberry Pi with an onboard mjbots [pi3hat](https://mjbots.com/products/mjbots-pi3hat-r4-4b). Servos are stopped when the spine is stopped, and switch to [position mode](https://github.com/mjbots/moteus/blob/main/docs/reference.md#theory-of-operation) (which is a position-velocity-torque controller) when the spine idles.

Check out the [spine state machine](https://tasts-robots.org/doc/vulp/classvulp_1_1spine_1_1StateMachine.html#details) for a summary of what "stop" and "idle" mean in this context.
