# Developer notes {#dev-notes}

## From newbie to sourcie

While newcomers are expected to install the ``upkie`` package from conda-forge or PyPI, as you get acquainted with the robot and start developing your own features you will probably want to start compiling from source. Here is a short guide

### Simulation

The simulation script will run pre-compiled binaries if possible:

```
./start_simulation.sh
```

To make sure you build and run the simulation from source, you can call the equivalent Bazel instruction:

```
./tools/bazelisk run //spines:bullet_spine -- --show
```

Bazel is the build system behind Upkie.

### Python module

There are two ways to make Python import your local ``upkie`` module when you are updating its source code. One of them is to re-install the package locally, using ``pip`` from the root of (your local copy of) the repository:

```
cd upkie
pip install -U .
```

Another one is to simply link the module from the directory you are working in. For instance, in the examples directory:

```
cd upkie/examples
ln -s ../upkie ./
```
