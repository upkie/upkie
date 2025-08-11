# Simulation {#simulation}

[TOC]

## Bullet spine {#bullet-spine}

The Bullet spine runs agents in the [Bullet 3](https://github.com/bulletphysics/bullet3) simulator. It can be started as a standalone process that will keep on running while waiting for agents to connect.

The easiest way to start a simulation spine is to run the simulation script from the repository:

<img src="bullet-spine.png" height="100" align="right" />

```console
./start_simulation.sh
```

The script will run pre-compiled binaries, downloading them from the latest release if necessary.

### Building from source

To build and run the simulation spine from source, [setup your build environment](\ref setup-build), then run the following Bazel command:

```console
./tools/bazelisk run //spines:bullet_spine -- --show
```

Bazel is the build system used in Upkie for spines. You can check out simulation options by appending the help flag `-h` to the above command.
