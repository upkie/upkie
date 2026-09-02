# 5) Motion control software {#motion-control-software}

[TOC]

The prerequisites for this stage are:

- Local copy of the main `upkie` repository from \ref getting-started
- Electronics setup from \ref electronics-testing
- USB controller from the [bill of materials](\ref bill-of-materials)

## 5.1) Go to the main repository

Change directory to your local copy of the `upkie` repository:

```console
$ cd upkie
```

In the following instructions, we will run everything from this directory.

## 5.2) Configure servo limits

Go to the tools directory on the robot and run the servo configuration script:

```console
pi@upkie:~$ cd tools
pi@upkie:tools$ ./configure_servos
```

Check that the output lines are only `OK` or `AOK`.

## 5.3) Build the spine

In your local copy of `upkie`, build the [pi3hat spine](\ref pi3hat-spine) for the robot by:

```console
$ make build
```

Running this for the first time requires an Internet connection.

## 5.4) Upload to the robot

Connect to the robot's Wi-Fi network, then from the `upkie` directory run:

```console
$ make upload
```

This will create a remote directory (also named `upkie`) on the Raspberry Pi with a copy of all the build files necessary to run the spine and agents on the robot.

This rule connects to the `upkie` host from your SSH configuration, which we set up in \ref raspberry-pi-setup. If you use a different nickname for your robot, run `make upload UPKIE_HOST=your_nickname` instead.

## 5.5) Connect the USB controller

Make sure the USB controller is connected to the Raspberry Pi. The controller has two functions:

1. Communicate commands to the robot, for instance using the joysticks to move around.
2. *Emergency stop:* pressing the right button (red button 🔴 on an Xbox controller) shuts down all motors immediately.

Always keep the emergency stop around when you run a new agent on the robot, and don't hesitate to press it if the beast misbehaves 😉

## 5.6) Start the pi3hat spine

Connect via SSH to the robot and start the pi3hat spine:

```console
pi@upkie:~$ pi3hat_spine
```

The spine is a fast program in charge of two-way communications between your agent's code (for us it will be the joystick controller here) and the robot's actuators.

## 5.7) Run a balancing agent

Take the Raspberry Pi in your hand (only touching the sides of the board) and put it upside-down (as it will be when the assembled robot stands upright). In a separate shell on the robot, run the joystick controller from the `upkie` directory:

```console
pi@upkie:upkie$ pixi run real-follow-joystick
```

Rotate the Raspberry Pi in your hands: the qdd100 servos should stay still while the two mj5208 servos turn in opposite directions.

<sub>Ask questions about this step in [Software discussions](https://github.com/upkie/upkie/discussions/categories/software).</sub>

---

**Previous:** \ref electronics-testing — **Next:** \ref assembly
