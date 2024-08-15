# PID balancer

The PID balancer is a baseline agent designed to test an Upkie quickly. It has minimum dependencies beyond the main `upkie` module.

## Usage

- Start a simulation or real-robot [spine](https://upkie.github.io/upkie/spines.html)
- Run the agent: `python pid_balancer/main_standalone.py`

## Overview

This agent balances the robot by PI feedback from torso-pitch and ground-position to wheel velocities (`WheelController`), keeping the legs' hip and knee joints straight (`ServoController`). The wheel controller adds a feedforward [non-minimum phase trick](https://github.com/upkie/upkie/blob/513fea81673f89646fdffcbad2f65ca9a0941ca6/pid_balancer/wheel_controller.py#L433-L457) for smoother transitions from standing to rolling.
