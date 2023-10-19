# Spines {#spines}

Upkie's code is organized into *spines*, which communicate with the simulation or mjbots actuators, and *agents*, the programs that implement robot behaviors. We use [Bazel](https://bazel.build/) to build spines, both for simulation on your development computer or for running on the robot's Raspberry Pi. Bazel does not install anything on your system: it fetches dependencies with specific versions and builds them locally, making sure the code stays consistent over time. Check out [this introduction](https://github.com/tasts-robots/vulp#readme) for more details.

## Bullet spine {#bullet-spine}

The Bullet spine runs an agent in the [Bullet 3](https://github.com/bulletphysics/bullet3) simulator. We can start this spine as a standalone process and let it run waiting for agents to connect:

```console
./start_simulation.sh
```

This script is just an alias for a Bazel ``run`` command:

```console
./tools/bazelisk run //spines:bullet_spine -- --show
```

Check out simulation options (fly like an eagle!) with the help flag ``-h``.

## pi3hat spine {#pi3hat-spine}

The pi3hat spine is the one that runs on the real robot, where a [pi3hat r4.5](https://mjbots.com/products/mjbots-pi3hat-r4-5) is mounted on top of the onboard Raspberry Pi computer. To run this spine, we can either: download it from GitHub, or build it locally from source.

### Download the latest release

To get the latest pi3hat spine from GitHub, download the latest `pi3hat_spine` binary from [Continuous Integration build artifacts](https://github.com/tasts-robots/upkie/actions/workflows/bazel.yml), then `scp` it to the robot. TODO: make this more user-friendly ;)

```console
$ scp pi3hat_spine foo@robot
$ ssh foo@robot
foo@robot:~$ ./pi3hat_spine
```

Once the spine is running, you can run any agent in a separate shell on the robot, for example the wheel balancer:

```console
foo@robot:upkie$ make run_wheel_balancer
```

### Build from source

To build the pi3hat locally from source, then upload it to Raspberry Pi, use the Makefile at the root of the repository:

```console
make build
make upload UPKIE_NAME=your_upkie
```

Next, log into the Pi and run a pi3hat spine:

```console
$ ssh foo@robot
foo@robot:~$ cd upkie
foo@robot:upkie$ make run_pi3hat_spine
```

Once the spine is running, you can run any agent in a separate shell on the robot, for example the wheel balancer:

```console
foo@robot:upkie$ make run_wheel_balancer
```

## Mock spine {#mock-spine}

The mock spine is useful to run an agent on the robot without firing up the actuators. It works exactly as the pi3hat spine, replacing "pi3hat" with "mock" in all instructions above.
