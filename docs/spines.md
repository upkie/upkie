# Spines {#spines}

Upkie's code is organized into *spines*, which communicate with the simulation or mjbots actuators, and *agents*, the programs that implement robot behaviors. We use [Bazel](https://bazel.build/) to build spines, both for simulation on your development computer or for running on the robot's Raspberry Pi. Bazel does not install anything on your system: it fetches dependencies with specific versions and builds them locally, making sure the code stays consistent over time.

## Action-observation loop {#action-observation-loop}

Upkie implements an action-observation loop to control robots from a standalone "agent" process, like this:

\image html action-observation-loop.png
\image latex action-observation-loop.eps

The agent can be a simple Python script with few dependencies. This separation between agent and spine provides a robot/simulation switch to train or test agents in a simulation spine (such as the \ref bullet-spine below) before running them on the real system.

### Inter-process communication

More accurately, Upkie's software implements an inter-process communication (IPC) protocol implemented in the C++ and Python libraries. It is suitable for [real-time](https://en.wiktionary.org/wiki/real-time#English) but not high-frequency (> 1000 Hz) performance. This is fine for balancing and locomotion on Upkies, as there is [theoretical and empirical evidence](https://scaron.info/blog/balancing-is-a-low-frequency-task.html) suggesting they can balance themselves as leisurely as 20 Hz. And if you are wondering whether Python is suitable for real-time applications, we were too! Until we [tried it out](https://github.com/orgs/upkie/discussions/240).

All design decisions have their pros and cons. Here is a list of features and non-features to give you an overview of those we did in this project:

#### Features

- Run the same Python code on simulated and real robots
- Interfaces with to the [mjbots pi3hat](https://mjbots.com/products/mjbots-pi3hat-r4-4b) and mjbots actuators
- Interfaces with to the [Bullet](http://bulletphysics.org/) simulator
- Observer pipeline to extend observations
- Soft real-time: spine-agent loop interactions are predictable and repeatable
- Unit tested, and not only with end-to-end tests

#### Non-features

- Low frequency: designed for tasks that run in the 1–400 Hz range (like balancing)
- Soft rather than hard real-time guarantee: the code is empirically reliable by a large margin, that's it
- Weakly-typed IPC: typing is used within agents and spines, but the interface between them is only checked at runtime

## Simulation spines {#simulation-spines}

### Bullet spine {#bullet-spine}

The Bullet spine runs an agent in the [Bullet 3](https://github.com/bulletphysics/bullet3) simulator. We can start this spine as a standalone process and let it run waiting for agents to connect. The simulation script will run pre-compiled binaries if possible:

```console
./start_simulation.sh
```

To build and run the simulation from source, [setup your build environment](\ref setup-build) then call the equivalent Bazel instruction:

```console
./tools/bazelisk run //spines:bullet_spine -- --show
```

Bazel is the build system used in Upkie for spines. Check out simulation options by appending the help flag ``-h`` to the above command.

## Real-robot spines {#real-robot-spines}

### pi3hat spine {#pi3hat-spine}

The pi3hat spine is the one that runs on the real robot, where a [pi3hat r4.5](https://mjbots.com/products/mjbots-pi3hat-r4-5) is mounted on top of the onboard Raspberry Pi computer. To run this spine, you can either download it from GitHub, or build it locally from source.

#### Download the latest release

Assuming your robot is connected to the Internet, you can get the latest pi3hat spine from GitHub using the `upkie_tool` command-line utility:

```console
$ ssh user@upkie
user@upkie:~$ upkie_tool update
```

Once the spine is installed, start it from a command-line on the robot:

```console
user@upkie:~$ pi3hat_spine
```

You can then run any agent in a separate shell on the robot, for example the wheel balancer:

```console
user@upkie:upkie$ make run_pid_balancer
```

#### Build from source

To build the pi3hat locally from source, then upload it to Raspberry Pi, use the Makefile at the root of the repository:

```console
make build
make upload UPKIE_NAME=your_upkie
```

Next, log into the Pi and run a pi3hat spine:

```console
$ ssh user@upkie
user@upkie:~$ cd upkie
user@upkie:upkie$ make run_pi3hat_spine
```

Once the spine is running, you can run any agent in a separate shell on the robot, for example the wheel balancer:

```console
user@upkie:upkie$ make run_pid_balancer
```

### Mock spine {#mock-spine}

The mock spine is useful to run an agent on the robot without firing up the actuators. It works exactly as the pi3hat spine, replacing "pi3hat" with "mock" in all instructions above.
