# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [2.0.0] - 2023-10-26

### Added

- PPO balancer: Allow custom training path
- PPO balancer: Allow re-training with the same policy name
- PPO balancer: Low-pass filter action before acceleration clamping
- agents: Closed-loop model predictive control
- envs: Allow custom initial base velocity in Bullet config
- examples: Closed-loop model predictive control
- spines: Build and export the pi3hat spine binary in continuous integration (thanks to @pgraverdy)
- spines: ``--version`` flag for all spine binaries

### Changed

- MPC balancer: Remove ``asyncio`` logic
- PPO balancer: Remove ``asyncio`` logic
- Pink balancer: Remove ``asyncio`` logic
- Wheel balancer: Remove ``asyncio`` logic
- envs: Move reset state sampling to `InitRandomization` class
- spines: Add `_spine` suffix to binary names, e.g. `pi3hat_spine`
- spines: Allow pi3hat spine to run without joystick if user validates

### Fixed

- Make Makefile date command more portable (thanks to @boragokbakan)
- PPO balancer: Correct save frequency during training
- PPO balancer: Run policy deterministically after training
- envs: Merge default and runtime configuration dictionaries

### Removed

- **Breaking:** ``async_step`` function and ``asyncio`` logic
- **Breaking:** ``pi32_config`` as 64-bit is the new default
- **Breaking:** ``upkie.utils.log_path`` submodule and its utility function
- Dependency on mpacklog.cpp (already in Vulp)
- Dependency on mpacklog.py

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
- **Breaking:** Rename ``UpkieWheelsEnv`` to ``UpkieGroundVelocity``
- **Breaking:** Use `regulate_frequency` env kwarg instead of `frequency=None`
- Makefile: Default wheel balancer config to the output of `hostname`
- Makefile: Rename ``ROBOT`` environment variable to ``UPKIE_NAME``
- PPO balancer: Change training directory to `/tmp/ppo_balancer`
- PPO balancer: Policy CLI argument becomes positional and optional
- PPO balancer: Refactor agent settings
- PPO balancer: Rename `effective_time_horizon` to `discounted_horizon_duration`
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

- Makefile: Remove most agent targets to promote running their ``main.py``
- PPO balancer: Can now be run both via Bazel or Python
- Pink balancer: Can now be run both via Bazel or Python
- Pink balancer: Remove unit tests
- README: Recommended way to run agents is now via Python
- Wheel balancer: Can now be run both via Bazel or Python
- envs: Default reward for all environments is now the survival reward
- envs: Move `StandingReward` to the PPO balancer
- examples: Remove CPU isolation example, now a ``utils.raspi`` function call
- tools: CPU scaling scripts don't need to be run as root any more
- utils: Remove ``realtime`` submodule in favor of ``raspi``

### Fixed

- PPO balancer: Configure main script process on the Raspberry Pi
- PPO balancer: Disable rate limiter during training
- Wheel balancer: Handle SpineInterface failures when forking a Bullet simulation (thanks to @boragokbakan)
- observers: Check whether floor contact observer is initialized properly
- tools: Fix permissions of `vcgencheck`

## [1.3.4] - 2023-08-09

### Added

- envs: Inform user on installing all optional dependencies
- envs: Name environment rate limiters for more readable logging

### Changed

- envs: Refactor environment registration function

### Fixed

- PyPI: add PyYAML to dependencies as it is needed by ``upkie.config``
- envs: Export ``register`` from submodule
- envs: Overlay constructor spine configuration on top of default config

## [1.3.3] - 2023-08-07

### Changed

- PyPI: Bump scipy dependency from 1.8.0 to 1.10.0
- envs: Skip environment registration upon missing dependency

## [1.3.2] - 2023-08-07

### Fixed

- config: Distribute missing ``config`` submodule in PyPI package

## [1.3.1] - 2023-07-28

### Fixed

- config: Add missing ``config`` submodule to PyPI
- Fix source code distribution of PyPI package

## [1.3.0] - 2023-07-26

### Added

- Support macOS operating systems (thanks to @boragokbakan)
- config: Top-level configuration submodule for robot-wide configuration
- spines: Build and deploy the mock spine to the Raspberry Pi
- tools: CPU frequency scaling scripts
- utils: Function to detect when we run on a Raspberry Pi

### Changed

- deps: Update loop-rate-limiters to 0.5.0
- envs: Rename ``config`` parameter to a more explicit ``spine_config``
- envs: Update to the Gymnasium API (thanks to @perrin-isir)

### Fixed

- PPO balancer: Fix time-limit import
- envs: Add ``dt`` attribute to the base environment

## [1.2.1] - 2023-07-18

### Changed

- envs: Bump UpkieWheelsEnv version number to 4

## [1.2.0] - 2023-07-18

### Added

- tools: Add ``vcgencheck`` utility script
- tools: Make ``hard_rezero`` search for ``upkie_tool`` and skip if not found

### Changed

- build: Compile in optimized mode by default (previsouly: fast build)
- envs: Observation vector reordered, angular velocity is now that of the base.

### Fixed

- build: Only run lint tests when ``--config lint`` is supplied
- envs: Make sure vectorized observations are float32

## [1.1.0] - 2023-07-07

### Added

- agents: Detect config from hostname when running on the Pi
- envs: Allow faster-than-realtime Gym environment execution (thanks to @perrin-isir)
- envs: Getter for environment frequency

### Changed

- agents: Rename "test balancer" to "wheel balancer"
- agents: Update Pink balancer to the latest version of the library
- Makefile: Check that ``ROBOT`` environment variable is defined
- tools: Automatically ``sudo`` when running ``upkie_tool``

### Fixed

- observers: Add missing observer configuration in base Upkie environment
- observers: Configure IMU orientation in the base pitch observer
- spines: Fix joint IDs to left leg (1, 2, 3), right leg (4, 5, 6)
- workspace: Update Vulp for IMU frame simulation fix
- workspace: Update ``upkie_description`` for IMU orientation fix

## [1.0.0] - 2023-06-12

### Added

- Example: lying genuflections
- Utils: base class and Upkie-specific exceptions

### Changed

- Environment: ``UpkieServosEnv-v2`` with frequency regulation
- Environment: ``UpkieWheelsEnv-v3`` with frequency regulation
- Environment: regulate loop frequencies
- Rename ``ROBOT_NAME`` to ``ROBOT`` in the main Makefile
- Rename main repository and project to just "upkie"
- Update Vulp to v1.2.0

## [0.5.0] - 2023-06-05

### Added

- Example: wheeled balancing with CPU isolation
- Example: wheeled balancing with action-observation logging
- Makefile to build, upload and run Raspberry Pi targets
- Script to start a simulation directly from the repository
- Specify Bazel version in ``.bazelversion``

### Changed

- Remove ``utils.logging`` submodule, deprecated by mpacklog
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

- Environment: ``UpkieServosEnv-v1``
- PPO balancer: distribute sample policy
- Pink balancer: ``--visualize`` argument
- Spine: Air Bullet, same as Bullet but floating in the air
- Utils: Pinocchio utility functions

### Changed

- Load description from local package rather than cloning a repository
- Rename ``UpkieWheelsReward`` to ``StandingReward``

### Fixed

- UpkieWheelsEnv: observation limits for ground position and velocity
- Pink balancer: reduce LM damping to fix stalling when crouching
- PPO balancer: get environment id (thanks @Varun-GP)

## [0.3.1] - 2023-03-15

### Added

- UpkieWheelsEnv: return action and observation dicts in ``info``

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
- Environment: ``UpkieWheelsEnv-v1``
- Observers: Base pitch

### Changed

- Promote ``utils.imu`` to a proper Python observer
- Switch from ``aiorate`` to ``loop_rate_limiters``

### Fixed

- Document both C++ and Python code with Doxygen
- Python requirements and dependencies in Bazel

## [0.1.0] - 2022-09-12

Starting this changelog.

[unreleased]: https://github.com/tasts-robots/upkie/compare/v2.0.0...HEAD
[2.0.0]: https://github.com/tasts-robots/upkie/compare/v1.5.0...v2.0.0
[1.5.0]: https://github.com/tasts-robots/upkie/compare/v1.4.0...v1.5.0
[1.4.0]: https://github.com/tasts-robots/upkie/compare/v1.3.4...v1.4.0
[1.3.4]: https://github.com/tasts-robots/upkie/compare/v1.3.3...v1.3.4
[1.3.3]: https://github.com/tasts-robots/upkie/compare/v1.3.2...v1.3.3
[1.3.2]: https://github.com/tasts-robots/upkie/compare/v1.3.1...v1.3.2
[1.3.1]: https://github.com/tasts-robots/upkie/compare/v1.3.0...v1.3.1
[1.3.0]: https://github.com/tasts-robots/upkie/compare/v1.2.1...v1.3.0
[1.2.1]: https://github.com/tasts-robots/upkie/compare/v1.2.0...v1.2.1
[1.2.0]: https://github.com/tasts-robots/upkie/compare/v1.1.0...v1.2.0
[1.1.0]: https://github.com/tasts-robots/upkie/compare/v1.0.0...v1.1.0
[1.0.0]: https://github.com/tasts-robots/upkie/compare/v0.5.0...v1.0.0
[0.5.0]: https://github.com/tasts-robots/upkie/compare/v0.4.0...v0.5.0
[0.4.0]: https://github.com/tasts-robots/upkie/compare/v0.3.1...v0.4.0
[0.3.1]: https://github.com/tasts-robots/upkie/compare/v0.3.0...v0.3.1
[0.3.0]: https://github.com/tasts-robots/upkie/compare/v0.2.0...v0.3.0
[0.2.0]: https://github.com/tasts-robots/upkie/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/tasts-robots/upkie/releases/tag/v0.1.0
