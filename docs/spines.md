# Spines {#spines}

[TOC]

There are two real-robot spines: the mock spine, and the pi3hat spine.

## Bullet spine {#bullet-spine}

The Bullet spine runs agents in the [Bullet 3](https://github.com/bulletphysics/bullet3) simulator. It can be started as a standalone process that will keep on running while waiting for agents to connect.

The easiest way to start a simulation spine is to run the simulation script from the repository:

<img src="bullet-spine.png" height="100" align="right" />

```console
./start_simulation.sh
```

The script will run pre-compiled binaries, downloading them from the latest release if necessary.

## Mock spine {#mock-spine}

The mock spine is useful to run an agent on the robot without firing up the actuators. It works exactly as the pi3hat spine, replacing "pi3hat" with "mock" in all instructions.

## Pi3hat spine {#pi3hat-spine}

The pi3hat spine is the one that runs on the real robot, where a [pi3hat r4.5](https://mjbots.com/products/mjbots-pi3hat-r4-5) is mounted on top of the onboard Raspberry Pi computer. To run this spine, you can either download it from GitHub, or [build it locally from source](\ref cpp-dev-workflow).

To download the latest release, assuming your robot is connected to the Internet, you use the `upkie_tool` command-line utility:

```console
$ ssh user@upkie
user@upkie:~$ upkie_tool update
```

Alternatively, you can manually go to the [Release page](https://github.com/upkie/upkie/releases), download `pi3hat_spine` from the assets of the latest release and `scp` it to `/usr/local/bin` on your robot.

Once the spine is installed, start it from the command line:

```console
user@upkie:~$ pi3hat_spine
```

You can then run any agent in a separate shell on the robot, for example the PID balancer from the examples directory:

```console
user@upkie:upkie$ python -m agents.mpc_balancer
```
