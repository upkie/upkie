# Upkie locomotion

![C++ version](https://img.shields.io/badge/C++-17/20-blue.svg?style=flat)
[![Build](https://img.shields.io/github/workflow/status/tasts-robots/upkie_locomotion/build)](https://github.com/tasts-robots/upkie_locomotion/actions/workflows/build.yml)
[![Test](https://img.shields.io/github/workflow/status/tasts-robots/upkie_locomotion/test?label=test)](https://github.com/tasts-robots/upkie_locomotion/actions/workflows/test.yml)
[![Coverage](https://coveralls.io/repos/github/tasts-robots/upkie_locomotion/badge.svg?branch=main)](https://coveralls.io/github/tasts-robots/upkie_locomotion?branch=main)
[![Documentation](https://img.shields.io/badge/docs-online-brightgreen?logo=read-the-docs&style=flat)](https://tasts-robots.org/doc/upkie_locomotion/)
[![Vulp](https://img.shields.io/badge/%F0%9F%A6%8A%20vulp-1.0.0-orange)](https://github.com/tasts-robots/vulp)

Collection of Python agents, observers and Vulp spines for the [Upkie](https://hackaday.io/project/185729-upkie-wheeled-biped-robot) wheeled biped. 🚧 **Pre-release.**

### Try it out!

<!-- GIF: https://user-images.githubusercontent.com/1189580/170491850-dfbb4786-12ff-4fe8-8080-9413d68acfc1.gif -->
<!-- Issue: https://github.com/github/feedback/discussions/17256 -->
<img src="https://user-images.githubusercontent.com/1189580/170496331-e1293dd3-b50c-40ee-9c2e-f75f3096ebd8.png" height="100" align="right" />

```console
git clone https://github.com/tasts-robots/upkie_locomotion.git
cd upkie_locomotion
./tools/bazelisk run -c opt //agents/blue_balancer:bullet
```

Connect a USB controller to move the robot around. 🎮

There is no dependency to install on Linux thanks to [Bazel](https://bazel.build/), which builds all dependencies and runs the Python controller in one go. (This will take a while the first time.) The syntax is the same to deploy to the Raspberry Pi of the living-room version of the robot.

## Contents

The code is organized into *spines*, which communicate with the simulator (Bullet) or actuators (pi3hat on the Raspberry Pi) using the [Vulp](https://github.com/tasts-robots/vulp) C++ library, and *agents*, the main programs that implement behaviors in Python.

Additionally, spines can be extended with *controllers* and *observers* to transfer functionality from Python "thoughts" to C++ "reflexes". In the case of Upkie, there is no additional controller (actuators are commanded directly from Python) but there are additional observers to detect contacts and keep track of where the wheels are on the ground.

### Agents

#### 🔵 Blue balancer

A 200 Hz agent designed to check out Upkie's physical capabilities. It is repeatable, and a good entry point for newcomers. It balances the robot using PID feedback from the head's pitch to wheel velocities, plus a feedforward [non-minimum phase trick](https://github.com/tasts-robots/upkie_locomotion/blob/55a331c6a6a165761a85087b7bea35d1403a6cf9/agents/blue_balancer/wheel_balancer.py#L368) for smoother transitions from standing to rolling. An analytical inverse kinematics is also plugged in for crouching and standing up. (It is connected to the D-pad of the USB controller if one is found.)

#### 🟣 Pink balancer

Same as the Blue balancer, but inverse kinematics is computed by [Pink](https://github.com/tasts-robots/pink) rather than with a model-specific analytical solution. This is the controller that runs in the [first](https://www.youtube.com/shorts/8b36XcCgh7s) [two](https://www.youtube.com/watch?v=NO_TkHGS0wQ) videos of Upkie.

### Observers

* [Floor contact](https://tasts-robots.org/doc/upkie_locomotion/classupkie__locomotion_1_1observers_1_1FloorContact.html#details): Observe contact between the wheels and the floor.
* [Wheel contact](https://tasts-robots.org/doc/upkie_locomotion/classupkie__locomotion_1_1observers_1_1WheelContact.html#details): ...
* Wheel odometry: ...

### Spines

* Bullet: ...
* pi3hat: ...
