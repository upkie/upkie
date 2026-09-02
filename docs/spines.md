# Spines {#spines}

[TOC]

## Bullet spine {#bullet-spine}

The Bullet spine runs agents in the [Bullet 3](https://github.com/bulletphysics/bullet3) simulator. It can be started as a standalone process that will keep on running while waiting for agents to connect.

The easiest way to start a simulation spine is to run the simulation script from the repository:

<img src="bullet-spine.png" height="100" align="right" />

```console
./start_simulation.sh
```

The script will run pre-compiled binaries, downloading them from the latest release if necessary.

## Pi3hat spine {#pi3hat-spine}

The pi3hat spine is the one that runs on the real robot, where a [pi3hat r4.5](https://mjbots.com/products/mjbots-pi3hat-r4-5) is mounted on top of the onboard Raspberry Pi computer. To run this spine, you can either download it from GitHub, or [build it locally from source](\ref cpp-dev-workflow).

To install the latest release, go to the [Release page](https://github.com/upkie/upkie/releases), download `pi3hat_spine` from the assets of the latest release, `scp` it to your robot and install it to `/usr/local/bin`:

```console
$ scp pi3hat_spine pi@upkie:
$ ssh pi@upkie
pi@upkie:~$ sudo install --owner=root --group=root --mode=4755 pi3hat_spine /usr/local/bin/pi3hat_spine
```

The spine needs root privileges to run in the real-time scheduler and talk to the pi3hat, hence the setuid bit (`4` in `4755`) that allows regular users to run it. If you [build the spine from source](\ref cpp-dev-workflow), running `make install_pi3hat_spine` from the repository installs it the same way.

Once the spine is installed, start it from the command line:

```console
pi@upkie:~$ pi3hat_spine
```

You can then run any agent in a separate shell on the robot, for example the PID balancer from the examples directory:

```console
pi@upkie:upkie$ python -m mpc_balancer
```

### Read-only mode {#readonly-mode}

The pi3hat spine can run in read-only mode, which communicates with the actuators to read their state but won't send position commands. This is useful to run agents over real robot states:

```console
pi@upkie:~$ pi3hat_spine --readonly
```

### Mock mode {#mock-mode}

The pi3hat spine can run in mock mode, which is useful to run an agent on the robot without communicating with the actuators:

```console
pi@upkie:~$ pi3hat_spine --mock
```
