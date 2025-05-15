# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Development workflow based on Pixi
- cpp: Controller pipeline run by the spine after observers

### Changed

- cpp: Spines now take a controller pipeline as constructor argument

### Fixed

- CICD: Update release workflow images to `ubuntu-latest`

## [8.0.0] - 2025-05-08

#### Added

- docs: Install `libtinfo5` when building from source on Debian-based distros
- examples: Domain randomization by environment wrapping
- sensors: Sensor pipeline that runs at every spine cycle

### Changed

- **Breaking:** envs: Make `UpkieGroundVelocity` a wrapper around `UpkieServos`
- **Breaking:** envs: Merge base and servos environments
- CICD: Setup Micromamba in coverage job
- Report error messages in simulation script
- Update minimum Gymnasium version to 1.0
- Update palimpsest to version 2.3.2
- cpp: Spine constructor now takes an additional sensor-pipeline argument
- envs: Hard-code spine retries to ten attempts

### Fixed

- examples: Fix warning in model predictive control example
- Spine: Fix logging when substepping a simulation spine

### Removed

- envs: Remove unused `WheeledInvertedPendulum` environment

## [7.0.0] - 2025-03-10

### Added

- Add pixi-pack integration
- actuation: Add collision-with-environment observation (thanks to @Tordjx)
- docs: Start Kinematics page
- utils: Add `clear_shared_memory` utility function

### Changed

- Bazel: Treat warnings as errors (except the one we can't avoid)
- Minimum Python version is now 3.9
- envs: Observation-based reward wrapper
- envs: Unit test for the base `step` function
- examples: Group simulation-related examples in a sub-directory

### Fixed

- Bazel: Ignore `.pixi` directory as it can contain unrelated Bazel files
- Fix unused variable warning in Bullet interface

### Removed

- **Breaking:** Remove `get_reward` functions from all environments
- Makefile: Remove conda packing rules, now deprecated in favor of Pixi

## [6.1.0] - 2024-12-12

### Added

- Configure Pixi in `pyproject.toml`
- Handle GLIBC version incompatibility in `start_simulation.sh`
- Spine: Throw an exception when a servo reply has invalid torque
- Update downloaded simulation spine in cache if outdated
- actuation: Added link inertia randomization to the Bullet interface (thanks to @Tordjx)
- actuation: Check maximum torques before sending commands
- cpp: Add `ServoError` exception
- model: Static model for joints with position, velocity and torque limits
- pixi: Task to generate the documentation by `pixi -e docs make`

### Changed

- CICD: Build release spines with Ubuntu 20.04 rather than 22.04
- CICD: Switch to Pixi for documentation workflow
- CICD: Update upload-artifact action to v4
- Remove layout argument to actuation interfaces
- actuation: Rename `Interface::initialize_action` to `Interface::reset_action`
- actuation: Rename `Interface::servo_joint_map` to `Interface::servo_name_map`
- deps: Upate to palimpsest 2.2.1

### Fixed

- examples: Lying genuflection example (thanks to @one-for-all)
- examples: Unwrap environments to get their neutral actions
- Update `start_simulation.sh` for systems where `-v` is not defined

### Removed

- env: Remove unused `leg_return_period` and hard-code it to one second
- env: Replace contingent `parse_first_observation` by a reset override
- model: Remove C++ `upkie::model` namespace
- observers: Remove upper-leg and wheel joints from configurable parameters

## [6.0.0] - 2024-11-01

### Added

- Makefile: Conda environment packing and unpacking rules
- Support Python 3.12
- tools: Add `ps_thread` command

### Changed

- **Breaking:** Spine and spine interface revisions:
    - Observations are now returned upon reset and step
    - Spine: Remove separate observation state and observation request
    - SpineInterface: Setting an action now returns an observation
    - SpineInterface: Starting the spine now returns an observation
    - StateMachine: Rename spine FSM state from "act" to "step"
    - docs: Update spine FSM specification in the documentation
- **Breaking:** actuation: Move resolution to static configuration
- **Breaking:** actuation: Move servo layout to static configuration
- **Breaking:** envs: Rename `LowPassFilterAction` to `AddLagToAction`
- CICD: Pin Ubuntu workflows to 22.04
- Default `kd` gain for wheel servos is now 0.3
- Move Python spine exceptions to `upkie.exceptions`
- docs: Turn environment page into an index
- tools: Make output directory an argument in `dump_servo_configs`
- tools: Simplify servo configuration script

### Fixed

- BulletInterface: Fix typo in a comment
- CICD: Install Doxygen with specific version from conda-forge
- Spine: Fix observation consistency between `run` and `simulate`
- envs: Clamp ground velocity action in `UpkieGroundVelocity`

### Removed

- Bazel: Remove legacy rules for PyPI dependencies
- Move PID balancer to its own repository
- deps: Remove dependency on PyYAML
- examples: Remove Bazel BUILD file
- model: Remove duplicate maximum torque constant

## [5.2.0] - 2024-09-30

### Added

- BulletInterface: Apply custom initial joint configuration, if provided
- BulletInterface: Report velocity of the base link in groundtruth
- BulletInterface: Torque control noise to joint-property configuration
- BulletInterface: Torque measurement noise to joint-property configuration
- BulletInterface: Uncertainty on IMU accelerometer and gyroscope measurements
- Forward command-line arguments to the spine in `start_simulation.sh`
- ImuData: Add linear velocity field
- envs: Add a random push wrapper (thanks to @Tordjx)
- examples: Apply an external force to lift an Upkie in sim
- examples: Custom initial state with a non-zero joint configuration
- examples: Simulation with joint friction
- examples: Simulation with sensor noise
- spines: Add variant argument to BulletSpine
- CI: add nightly builds

### Changed

- BulletInterface: Move simulation body poses to `sim.bodies`
- BulletInterface: Rename internal unit-test getters
- actuation: Log simulation groundtruth to `sim`
- CI: Do not install Python packages in the system environment on runners

### Fixed

- BulletInterface: Fix application of external forces
- CICD: Enable stable-baselines3 environment-check unit tests
- docs: Add missing documentation pages
- docs: Document Gym environment wrappers
- envs: Handle spine errors raised in base env constructor
- envs: Register UpkieServoPositions and UpkieServoTorques environments

### Removed

- CICD: Remove pycodestyle as we now use ruff for Python linting
- BulletInterface: Remove unused orientation/position unit-test getters

## [5.1.0] - 2024-08-14

### Added

- Pi3HatInterface: Additional check on maximum torques for each command
- Pi3HatInterface: Log raw IMU measurements from pi3hat interface
- actuation: Extend ImuData with raw measurements
- envs: Add UpkieServoPositions child environment (thanks to @Tordjx)
- envs: Add UpkieServoTorques child environment (thanks to @Tordjx)
- envs: Add a `model` attribute to all Upkie environments
- envs: Start reward submodule with a wheeled inverted pendulum reward
- envs: Use wheel torque limits from model in `UpkieGroundVelocity`
- envs: Wheeled inverted pendulum environment for reduced-model testing
- exceptions: Add PositionCommandError exception
- model: Add `rotation_ars_to_world` and `rotation_base_to_imu`
- model: Add `upper_leg_joints` list
- tools: Add `update` command to `upkie_tool`
- utils: Add unit tests for Raspberry Pi utility functions
- utils: Raise an exception when trying to configure agent from an interpreter

### Changed

- Avoid using `numpy.typing` for compatibility with older versions of NumPy
- PID balancer: Fix dependencies for running on systems without conda
- PID balancer: Update filenames of the two main scripts
- actuation: Throw PositionCommandError rather than stopping a servo on error
- deps: Update to palimpsest 2.2.0
- envs: Refactor internal reward of `UpkieGroundVelocity` environment
- envs: Use torque limits from model in `UpkieGroundVelocity` environment
- examples: Make wheeled balancing example a bit more complex and more stable
- examples: wheeled inverted pendulum model example
- exceptions: Move to the top-level Python module

### Fixed

- BulletInterface: Correct inline of two Bullet utility functions
- BulletInterface: Fix IMU acceleration in Bullet upon resets
- envs: Make sure action values are floating-point numbers (thanks to @Tordjx)
- exceptions: Make all exceptions derive from UpkieError
- model: Fix type of upper-leg and wheel joint lists
- spine: Handle deserialization exceptions when beginning a cycle

### Removed

- MockInterface: Remove unused IMU data from mock interface

## [5.0.1] - 2024-08-01

### Fixed

- envs: Fix missing return value in fall detection function

## [5.0.0] - 2024-08-01

### Added

- Bazel: Add support for ARM64 CPUs
- CICD: Build jobs for x86 and ARM64 macOS spines
- CICD: Packaging job for conda-forge
- Import and adapt C++ code from Vulp (`vulp` namespace is now `upkie:cpp`)
- Log received actuation replies in spine cycles
- PID balancer: Conda environment file
- envs: Add `left_wheeled` parameter to the `UpkieGroundVelocity` environment
- examples: Tuning the gains of a standard two-task PI balancer
- model: Add joints submodule
- setup: Add micromamba installation script
- setup: Automate process of building new system images
- spine: Print out configuration dictionary upon reset
- spines: Expose environment body poses in Bullet spine (thanks to @Tordjx)
- tools: Configure servo gains during setup

### Changed

- **Breaking:** rename the default shared-memory file to `/upkie`
- Bazel: Update bazelisk version to 1.20.0
- CICD: Switch to Micromamba for unit testing
- Makefile: Separate rule to set the raspi date
- PID balancer: default to hostname for the agent configuration
- Put hostname before spine name in log file names
- deps: Update Upkie description to 2.1.0
- deps: Update pi3hat dependency to latest commit
- docs: Sort documentation pages by expected discovery steps
- envs: Warn when a fall is detected
- examples: Rename direct servo control example

### Fixed

- CICD: Release jobs for x86 and ARM64 macOS spines
- CICD: Update macOS x86 runner images (thanks to @ubgk)
- Pi3HatInterface: Fix duplicate `data_` attribute
- observers: Read configuration matrix in base orientation observer
- raspunzel: argv0 when executing the target is now the same as `bazel run`
- setup: Fix configuration-write order in servo config script

### Removed

- utils: Pinocchio utility functions
- deps: Dependency on separate Vulp project
- docs: Remove PID balancer from the documentation

## [4.0.0] - 2024-06-12

### Added

- PID balancer: Monitor both tire contacts in simulation examples
- docs: Developer notes
- envs: Add the `env.bullet_extra` function for magic actions in sim
- envs: Parameter to disable frequency checks during frequency regulation
- observers: Base orientation observer
- utils: Factor `get_log_path` function

### Changed

- **Breaking:** envs: Change API of logging function to `log(name, entry)`
- **Breaking:** envs: Restrict observation space of `UpkieServos` to servos
- deps: Update Vulp to 2.5.0
- envs: Bump `UpkieServos` version number to 4

### Fixed

- PID balancer: Fix shared-memory opening in standalone Bullet script
- dist: Exclude unnecessary files from Python packages
- envs: Stop the spine when deleting an environment instance

### Removed

- **Breaking:** observers: Python version of the base orientation observer
- Optional dependencies for balancers that now have their own repositories

## [3.4.0] - 2024-03-21

### Added

- Add `--build` argument to the simulation script
- CICD: Run ShellCheck on scripts
- Clear shared-memory when starting the Bullet spine
- Script to dump all servo motor-driver configurations

### Changed

- deps: Update Upkie description to 1.5.0
- deps: Update Vulp to 2.2.1
- Don't build simulation spine if execution fails
- Move agents' `requirements.txt` files to optional project dependencies
- palinode: Rename `run_pid_balancer.sh` to `try_pid_balancer.sh` 😊
- Rename top-level run script to `run_pid_balancer.sh`

### Fixed

- Fix Gymnasium API in the readme example (thanks to @araffin)
- Handle closing of GUI window in simulation script
- Make sure all `UpkieServos` box observations are proper arrays

### Removed

- Move MPC balancer to its own repository
- Move PPO balancer to its own repository

## [3.3.0] - 2024-02-20

### Added

- Script to run any of the standard agents
- spines: `bullet_spine` accepts extra URDFs as arguments (thanks to @ubgk)

### Changed

- deps: Bump loop-rate-limiters to 1.0.0
- deps: Replace posix-ipc by shared-memory from the standard library
- envs: Redirect logging to /env prefix
- envs: Rename default action in UpkieServos to neutral, its position is NaN
- Minimum Python version is now 3.8

### Removed

- Script to compile and run the PID balancer
- utils: ActionError exception

## [3.2.0] - 2024-02-08

### Added

- MPC balancer: Add classes and functions to the documentation
- PID balancer: Add classes and functions to the documentation
- PPO balancer: Add classes and functions to the documentation
- Robot state class to describe initial state distributions

### Changed

- CICD: update release GH action to package simulation spines on various architectures
- Code style: ignore E266 as Doxygen uses ## to document class attributes
- deps: Bump Vulp to 2.1.0
- envs: Default velocity in `UpkieServos` is now zero
- envs: Observation and action values are `float` rather than `np.float32`
- envs: Specify maximum torques in `UpkieGroundVelocity`
- palinode: Rename `try_pid_balancer.sh` to `start_pid_balancer.sh` 😊
- Simulation script downloads a binary if available, o/w compiles from source (thanks to @pgraverdy)

### Fixed

- envs: typo in `UpkieServos` dictionary key
- MPC balancer: add missing dependencies to requirements.txt
- PPO balancer: add missing initial state randomization to `--training` mode

### Removed

- Bazel: dependencies based on `pip_parse` from `rules_python`
- Pink balancer: moved to a separate repository

## [3.1.0] - 2023-12-22

### Added

- envs: Take feedforward torque commands into account in `UpkieServos`
- examples: Wheeled balancing with wheel torque control
- utils: ActionError exception
- utils: ModelError exception

### Changed

- Rename `start_pid_balancer.sh` to `try_pid_balancer.sh`
- envs: Make parsing of first observation optional for non-base environments
- envs: Rename info key to `"spine_observation"` in all environments
- envs: Switch `UpkieServos` env to dictionary action and observation

## [3.0.0] - 2023-12-01

### Added

- PPO balancer: Convenience Makefile to manage the training directory
- docs: Agents and Environments pages
- envs: Log action, observation and reward to "env" sub-dict of action
- envs: New `upkie.envs.wrappers` submodule
- envs: Wrapper to act on the discrete-derivative of an action
- envs: Wrapper to add the action to the observation vector
- envs: Wrapper to low-pass filter action (modeling a delay)
- envs: Wrapper to noisify actions
- envs: Wrapper to noisify observations
- logs: Convenience Makefile for log synchronization
- spines: Label pi3hat spine log filenames with the robot's hostname

### Changed

- **Breaking:** Rename the "wheel" balancer agent to "PID balancer"
- **Breaking:** envs: Rewards are now part of individual environments
- **Breaking:** envs: `info` dictionary does not repeat "action" any more
- MPC balancer: Update height of control point to 58 cm
- PPO balancer: Update height of control point to 58 cm
- deps: Bump Gymnasium to 0.29.1
- deps: Bump NumPy to 1.24.3
- deps: Bump Vulp to 2.0.0
- deps: Bump loop-rate-limiters to 0.6.1
- envs: Bump `UpkieGroundVelocity` to version 3
- envs: Legs now return to their neutral configuration in `UpkieGroundVelocity`
- envs: `rate` attribute from base environment is now internal
- examples: Refactor lying genuflections
- examples: Refactor wheeled balancing

### Removed

- docs: Code overview page, split into Agents, Environments and Observations
- envs: Acceleration filter from `UpkieGroundVelocity` (=> gymnasium.Wrapper)
- envs: Intermediate `UpkieWheeledPendulum` environment
- envs: Low-pass filter from `UpkieGroundVelocity` (=> gymnasium.Wrapper)
- envs: Survival reward, as environments now ship their own rewards

## [2.0.0] - 2023-10-26

### Added

- PPO balancer: Allow custom training path
- PPO balancer: Allow re-training with the same policy name
- PPO balancer: Low-pass filter action before acceleration clamping
- PPO balancer: Update reward to penalize commanded accelerations
- agents: Closed-loop model predictive control
- envs: Allow custom initial base velocity in Bullet config
- envs: Augment observation with previous command in `UpkieGroundVelocity`
- examples: Closed-loop model predictive control
- spines: Build and export the pi3hat spine binary in continuous integration (thanks to @pgraverdy)
- spines: `--version` flag for all spine binaries

### Changed

- MPC balancer: Remove `asyncio` logic
- PPO balancer: Remove `asyncio` logic
- Pink balancer: Remove `asyncio` logic
- Wheel balancer: Remove `asyncio` logic
- envs: Move reset state sampling to `InitRandomization` class
- spines: Add `_spine` suffix to binary names, e.g. `pi3hat_spine`
- spines: Allow pi3hat spine to run without joystick if user validates

### Fixed

- Make Makefile date command more portable (thanks to @ubgk)
- PPO balancer: Correct save frequency during training
- PPO balancer: Run policy deterministically after training
- envs: Merge default and runtime configuration dictionaries

### Removed

- **Breaking:** `async_step` function and `asyncio` logic
- **Breaking:** `pi32_config` as 64-bit is the new default
- **Breaking:** `upkie.utils.log_path` submodule and its utility function
- deps: Dependency on mpacklog.cpp (already in Vulp)
- deps: Dependency on mpacklog.py

## [1.5.0] - 2023-09-29

### Added

- Bazel: Python library target for upkie module
- PPO balancer: Save policy configuration to a YAML file
- PPO balancer: Train with multiprocessing
- envs: Accessor to `env.rate` for logging purposes
- envs: Randomize base orientation and position on resets
- envs: `InitRandomization` dataclass to describe initial state randomization
- envs: `UpkieGroundVelocity` can include a velocity low-pass filter
- envs: `UpkieGroundVelocity` can limit ground acceleration as well
- envs: `UpkieGroundVelocity` low-pass filter can also be randomized
- utils: Log path utility functions

### Changed

- **Breaking:** Environment rewards depend on both observation and action
- **Breaking:** Rename `UpkieServosEnv` to `UpkieServos`
- **Breaking:** Rename `UpkieWheelsEnv` to `UpkieGroundVelocity`
- **Breaking:** Use `regulate_frequency` env kwarg instead of `frequency=None`
- Makefile: Default wheel balancer config to the output of `hostname`
- Makefile: Rename `ROBOT` environment variable to `UPKIE_NAME`
- PPO balancer: Change training directory to `/tmp/ppo_balancer`
- PPO balancer: Policy CLI argument becomes positional and optional
- PPO balancer: Refactor agent settings
- agents: Retry connecting to the spine several times at startup
- envs: Retry connecting to the spine several times at startup

### Fixed

- Wheel balancer: Configure spine properly
- Wheel odometry: Check that observer is configured properly

### Removed

- envs: Remove `get_range` from rewards as it is deprecated from Gymnasium

## [1.4.0] - 2023-08-24

### Added

- Makefile: Add clean rule to remove all intermediate files
- Makefile: Add coverage rule to check a local HTML report
- agents: Add `--show` CLI argument to the wheel balancer's Bullet target
- envs: Base `Reward` class for rewards
- envs: Survival reward, which is simply always one
- utils: Configure agent process on the Raspberry Pi

### Changed

- Makefile: Remove most agent targets to promote running their `main.py`
- PPO balancer: Can now be run both via Bazel or Python
- Pink balancer: Can now be run both via Bazel or Python
- Pink balancer: Remove unit tests
- README: Recommended way to run agents is now via Python
- Wheel balancer: Can now be run both via Bazel or Python
- envs: Default reward for all environments is now the survival reward
- envs: Move `StandingReward` to the PPO balancer
- examples: Remove CPU isolation example, now a `utils.raspi` function call
- tools: CPU scaling scripts don't need to be run as root any more
- utils: Remove `realtime` submodule in favor of `raspi`

### Fixed

- PPO balancer: Configure main script process on the Raspberry Pi
- PPO balancer: Disable rate limiter during training
- Wheel balancer: Handle SpineInterface failures when forking a Bullet simulation (thanks to @ubgk)
- observers: Check whether floor contact observer is initialized properly
- tools: Fix permissions of `vcgencheck`

## [1.3.4] - 2023-08-09

### Added

- envs: Inform user on installing all optional dependencies
- envs: Name environment rate limiters for more readable logging

### Changed

- envs: Refactor environment registration function

### Fixed

- PyPI: add PyYAML to dependencies as it is needed by `upkie.config`
- envs: Export `register` from submodule
- envs: Overlay constructor spine configuration on top of default config

## [1.3.3] - 2023-08-07

### Changed

- PyPI: Bump scipy dependency from 1.8.0 to 1.10.0
- envs: Skip environment registration upon missing dependency

## [1.3.2] - 2023-08-07

### Fixed

- config: Distribute missing `config` submodule in PyPI package

## [1.3.1] - 2023-07-28

### Fixed

- config: Add missing `config` submodule to PyPI
- Fix source code distribution of PyPI package

## [1.3.0] - 2023-07-26

### Added

- Support macOS operating systems (thanks to @ubgk)
- config: Top-level configuration submodule for robot-wide configuration
- spines: Build and deploy the mock spine to the Raspberry Pi
- tools: CPU frequency scaling scripts
- utils: Function to detect when we run on a Raspberry Pi

### Changed

- deps: Update loop-rate-limiters to 0.5.0
- envs: Rename `config` parameter to a more explicit `spine_config`
- envs: Update to the Gymnasium API (thanks to @perrin-isir)

### Fixed

- PPO balancer: Fix time-limit import
- envs: Add `dt` attribute to the base environment

## [1.2.1] - 2023-07-18

### Changed

- envs: Bump UpkieWheelsEnv version number to 4

## [1.2.0] - 2023-07-18

### Added

- tools: Add `vcgencheck` utility script
- tools: Make `hard_rezero` search for `upkie_tool` and skip if not found

### Changed

- build: Compile in optimized mode by default (previsouly: fast build)
- envs: Observation vector reordered, angular velocity is now that of the base.

### Fixed

- build: Only run lint tests when `--config lint` is supplied
- envs: Make sure vectorized observations are float32

## [1.1.0] - 2023-07-07

### Added

- agents: Detect config from hostname when running on the Pi
- envs: Allow faster-than-realtime Gym environment execution (thanks to @perrin-isir)
- envs: Getter for environment frequency

### Changed

- agents: Rename "test balancer" to "wheel balancer"
- agents: Update Pink balancer to the latest version of the library
- Makefile: Check that `ROBOT` environment variable is defined
- tools: Automatically `sudo` when running `upkie_tool`

### Fixed

- observers: Add missing observer configuration in base Upkie environment
- observers: Configure IMU orientation in the base pitch observer
- spines: Fix joint IDs to left leg (1, 2, 3), right leg (4, 5, 6)
- workspace: Update Vulp for IMU frame simulation fix
- workspace: Update `upkie_description` for IMU orientation fix

## [1.0.0] - 2023-06-12

### Added

- Example: lying genuflections
- utils: base class and Upkie-specific exceptions

### Changed

- envs: `UpkieServosEnv-v2` with frequency regulation
- envs: `UpkieWheelsEnv-v3` with frequency regulation
- envs: regulate loop frequencies
- Rename `ROBOT_NAME` to `ROBOT` in the main Makefile
- Rename main repository and project to just "upkie"
- Update Vulp to v1.2.0

## [0.5.0] - 2023-06-05

### Added

- Example: wheeled balancing with CPU isolation
- Example: wheeled balancing with action-observation logging
- Makefile to build, upload and run Raspberry Pi targets
- Script to start a simulation directly from the repository
- Specify Bazel version in `.bazelversion`

### Changed

- Remove `utils.logging` submodule, deprecated by mpacklog
- Rename "blue balancer" agent to "test balancer"
- Simplify test balancer code
- Update Bazelisk to v1.16.0
- UpkieWheelsEnv: version 2

### Fixed

- Conversion warnings in servos and wheels environments
- Examples: make sure environments are closed before exiting
- PPO balancer: fix testing script
- PPO balancer: save policy with new environment
- PPO balancer: simplify training
- Pink balancer: default frequency is 200 Hz

## [0.4.0] - 2023-04-06

### Added

- envs: `UpkieServosEnv-v1`
- PPO balancer: distribute sample policy
- Pink balancer: `--visualize` argument
- spines: Air Bullet, same as Bullet but floating in the air
- utils: Pinocchio utility functions

### Changed

- Load description from local package rather than cloning a repository
- Rename `UpkieWheelsReward` to `StandingReward`

### Fixed

- UpkieWheelsEnv: observation limits for ground position and velocity
- Pink balancer: reduce LM damping to fix stalling when crouching
- PPO balancer: get environment id (thanks @Varun-GP)

## [0.3.1] - 2023-03-15

### Added

- UpkieWheelsEnv: return action and observation dicts in `info`

## [0.3.0] - 2023-03-13

### Added

- Add Gym environment registration function
- PPO balancer: setting for total number of training timesteps
- UpkieWheelsEnv: simple example

### Changed

- UpkieWheelsEnv: remove dependency on gin

## [0.2.0] - 2023-03-03

### Added

- Agent: PPO balancer
- envs: `UpkieWheelsEnv-v1`
- Observers: Base pitch

### Changed

- Promote `utils.imu` to a proper Python observer
- Switch from `aiorate` to `loop_rate_limiters`

### Fixed

- Document both C++ and Python code with Doxygen
- Python requirements and dependencies in Bazel

## [0.1.0] - 2022-09-12

Starting this changelog.

[unreleased]: https://github.com/upkie/upkie/compare/v8.0.0...HEAD
[8.0.0]: https://github.com/upkie/upkie/compare/v7.0.0...v8.0.0
[7.0.0]: https://github.com/upkie/upkie/compare/v6.1.0...v7.0.0
[6.1.0]: https://github.com/upkie/upkie/compare/v6.0.0...v6.1.0
[6.0.0]: https://github.com/upkie/upkie/compare/v5.2.0...v6.0.0
[5.2.0]: https://github.com/upkie/upkie/compare/v5.1.0...v5.2.0
[5.1.0]: https://github.com/upkie/upkie/compare/v5.0.1...v5.1.0
[5.0.1]: https://github.com/upkie/upkie/compare/v5.0.0...v5.0.1
[5.0.0]: https://github.com/upkie/upkie/compare/v4.0.0...v5.0.0
[4.0.0]: https://github.com/upkie/upkie/compare/v3.4.0...v4.0.0
[3.4.0]: https://github.com/upkie/upkie/compare/v3.3.0...v3.4.0
[3.3.0]: https://github.com/upkie/upkie/compare/v3.2.0...v3.3.0
[3.2.0]: https://github.com/upkie/upkie/compare/v3.1.0...v3.2.0
[3.1.0]: https://github.com/upkie/upkie/compare/v3.0.0...v3.1.0
[3.0.0]: https://github.com/upkie/upkie/compare/v2.0.0...v3.0.0
[2.0.0]: https://github.com/upkie/upkie/compare/v1.5.0...v2.0.0
[1.5.0]: https://github.com/upkie/upkie/compare/v1.4.0...v1.5.0
[1.4.0]: https://github.com/upkie/upkie/compare/v1.3.4...v1.4.0
[1.3.4]: https://github.com/upkie/upkie/compare/v1.3.3...v1.3.4
[1.3.3]: https://github.com/upkie/upkie/compare/v1.3.2...v1.3.3
[1.3.2]: https://github.com/upkie/upkie/compare/v1.3.1...v1.3.2
[1.3.1]: https://github.com/upkie/upkie/compare/v1.3.0...v1.3.1
[1.3.0]: https://github.com/upkie/upkie/compare/v1.2.1...v1.3.0
[1.2.1]: https://github.com/upkie/upkie/compare/v1.2.0...v1.2.1
[1.2.0]: https://github.com/upkie/upkie/compare/v1.1.0...v1.2.0
[1.1.0]: https://github.com/upkie/upkie/compare/v1.0.0...v1.1.0
[1.0.0]: https://github.com/upkie/upkie/compare/v0.5.0...v1.0.0
[0.5.0]: https://github.com/upkie/upkie/compare/v0.4.0...v0.5.0
[0.4.0]: https://github.com/upkie/upkie/compare/v0.3.1...v0.4.0
[0.3.1]: https://github.com/upkie/upkie/compare/v0.3.0...v0.3.1
[0.3.0]: https://github.com/upkie/upkie/compare/v0.2.0...v0.3.0
[0.2.0]: https://github.com/upkie/upkie/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/upkie/upkie/releases/tag/v0.1.0
