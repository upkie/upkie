# Simulation spines {#simulation-spines}

## Bullet spine {#bullet-spine}

The Bullet spine runs an agent in the [Bullet 3](https://github.com/bulletphysics/bullet3) simulator. We can start this spine as a standalone process and let it run waiting for agents to connect. The simulation script will run pre-compiled binaries if possible:

```console
./start_simulation.sh
```

To build and run the simulation from source, [setup your build environment](\ref setup-build) then call the equivalent Bazel instruction:

```console
./tools/bazelisk run //spines:bullet_spine -- --show
```

Bazel is the build system used in Upkie for spines. Check out simulation options by appending the help flag `-h` to the above command.
