# Developer notes {#dev-notes}

## From newbie to sourcie

While newcomers will likely run `start_simulation.sh` and import the `upkie` package in Python, as you get acquainted with the robot and develop your own agents (by forking [upkie/new\_agent](https://github.com/upkie/new_agent)), you may want to contribute some features back to the upstream repository. Here is a short guide on compiling from source to do that.

### Build environment setup {#setup-build}

First, setup a build environment following the instructions for your system:

- [Fedora](https://github.com/orgs/upkie/discussions/100)
- [macOs](https://github.com/orgs/upkie/discussions/159)
- [Ubuntu](https://github.com/orgs/upkie/discussions/101)

To upload software to the robot, you will also need to define the `UPKIE_NAME` environment variable. Assuming the hostname of your Upkie is "michel-strogoff", for example, you can add the following to your shell configuration file:

```
export UPKIE_NAME="michel-strogoff"
```

An IP address will also work.

To make sure your build environment works, try to run a [bullet spine](\ref bullet-spine) from source:

```
make run_bullet_spine
```

Once the simulation builds and runs successfully, move onto building the [pi3hat spine](\ref pi3hat-spine) from source, upload it and run it on your robot.

### Using the local Python module

If you make changes to the `upkie` Python module, you will want to make Python import your local version rather than the one installed from conda-forge or PyPI. There are two ways to do so. One of them is to re-install the package locally, using `pip` from the root of (your local copy of) the repository:

```
cd upkie
pip install -U .
```

Another one is to simply link the module from the directory you are working in. For instance, in the examples directory:

```
cd upkie/examples
ln -s ../upkie ./
```
