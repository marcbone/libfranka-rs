# Changelog

## [0.10.0] - Unreleased

This version supports Panda and Franka Research 3 robots.

Requires Franka Research 3 system version >= 5.2.0

Requires Panda system version >= 4.2.1

### Added

- Support for FR3 was added
- Most methods are now generic over the robot type, so they can work with FR3 and Panda robots
- Downloading of the robot model for other architectures
- New `RateLimiter` trait that is implemented by the `Panda` and `Fr3` structs
- The rate limiter for FR3 is using position based velocity limits
- Code coverage reports

### Changed

- The Robot struct became a trait that is implemented by the `Panda` and `Fr3` structs
- The Model struct became a trait that is implemented by the `RobotModel` struct
- Examples now default to FR3 unless `--panda` is passed as argument
- Rate limiter is disabled by default for FR3
- FR3 does not have get_virtual_wall and set_filters methods
- Updated `nalgebra` to version 0.32
- Updated `clap` to version 4

### Fixed

- Badges in README

## [0.9.0] - 2022-04-01

Requires Panda system version >= 4.2.1

### Added

- Add `O_ddP_O` base acceleration to robot state, hardcoded to `{0, 0, -9.81}`
- New `BaseAccelerationInitializationTimeout`, `BaseAccelerationInvalidReading`
  `CartesianSplineViolation` and
  `JointViaPlanLimitViolation` reflexes

### Changed

- `Model::gravity_from_state` uses `O_ddP_O` from the robot state

## [0.8.2-alpha-2] - 2021-06-18

Requires Panda system version >= 4.0.0

Initial release
