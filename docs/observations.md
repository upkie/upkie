# Observations {#observations}

Spines compute observation dictionaries from sensor measurements by applying *observers* one after the other in a sequence called the *observer pipeline*. This page lists the outputs of available observers, using shorthands `a.b.c` for nested dictionary keys `observation["a"]["b"]["c"]`.

## Inertial measurement unit

| Observation key | Description |
|-----------------|-------------|
| `imu.angular_velocity` | [Body angular velocity](\ref upkie::cpp::actuation::ImuData::angular_velocity_imu_in_imu) of the IMU frame in [rad] / [s] |
| `imu.linear_acceleration` | [Linear acceleration](\ref upkie::cpp::actuation::ImuData::linear_acceleration_imu_in_imu) of the IMU, with gravity filtered out, in [m] / [s]² |
| `imu.orientation` | [Orientation of the IMU frame](\ref upkie::cpp::actuation::ImuData::orientation_imu_in_ars) in the [ARS](\ref ars) frame as a unit quaternion (w, x, y, z) |
| `imu.raw_angular_velocity` | [Raw angular velocity](\ref upkie::cpp::actuation::ImuData::raw_angular_velocity) measured by the gyroscope of the IMU, in [rad] / [s] |
| `imu.raw_linear_acceleration` | [Raw linear acceleration](\ref upkie::cpp::actuation::ImuData::raw_linear_acceleration) measured by the accelerometer of the IMU, in [m] / [s]² |

The inertial measurement unit (IMU) mounted on the [pi3hat](https://mjbots.com/products/mjbots-pi3hat-r4-5) combines an accelerometer and a gyroscope. These raw measurements are converted onboard by an unscented Kalman filter (based on a standard quasi-static assumption) that outputs observed quantities with respect to an attitude reference system (ARS) frame.

Upkie spines always report [IMU observations](\ref upkie::cpp::actuation::ImuData).

### World frame {#world-frame}

The world frame is an inertial frame of reference we refer to in various observers. It has an x-axis pointing forward, a y-axis pointing to the left and a z-axis vertical and pointing up (*i.e.*, against the gravity vector).

### Attitude reference system {#ars}

The attitude reference system (ARS) frame is an inertial frame of reference used by the the IMU filte. It has its x-axis pointing forward, y-axis pointing to the right and z-axis pointing down ([details](https://github.com/mjbots/pi3hat/blob/ab632c82bd501b9fcb6f8200df0551989292b7a1/docs/reference.md#orientation)). This is not the convention we use in the world frame, and the rotation matrix from the ARS frame to the world frame is:

\f$
R_{WA} = \begin{bmatrix}
    1 & 0 & 0 \\
    0 & -1 & 0 \\
    0 & 0 & -1 \\
\end{bmatrix}
\f$

## Servo actuators

| Observation key | Description |
|-----------------|-------------|
| `servo.X` | Observations for servo `X` in the servo layout |
| `servo.X.position` | Angle between the stator and the rotor in [rad] |
| `servo.X.torque` | Joint torque in [N m] |
| `servo.X.velocity` | Angular velocity of the rotor w.r.t. stator in rotor, in [rad] / [s] |

Actuators on the robot are [qdd100 beta 3](https://mjbots.com/products/qdd100-beta-3) (hips and knees) and [mj5208 brushless motors](https://mjbots.com/products/mj5208) (wheels). All of them have [moteus](https://mjbots.com/products/moteus-r4-11) controller boards that provide the above measurements. They are estimated as follows:

- Joint angle, sensed by two orthogonal Hall-effect sensors at the back of the moteus controller board (each measuring the magnetic field in one direction; the output angle is then the arc-tangent of the ratio between these two values)
- Joint velocity, obtained by filtering joint angle measurements (not a direct measurement)
- Joint torque, estimated from sensed phase currents (with a model that includes [stator magnetic saturation](https://jpieper.com/2020/07/31/dealing-with-stator-magnetic-saturation/))

Upkie spines always report servo observations.

## Observers {#observers}

### Base orientation

| Observation key | Description |
|-----------------|-------------|
| `base_orientation.angular_velocity` | Body angular velocity vector of the base frame in [rad] / [s] |
| `base_orientation.pitch` | Pitch angle of the base frame relative to the world frame, in radians |

The [BaseOrientation](\ref upkie::cpp::observers::BaseOrientation) observer estimates the orientation of the floating base with respect to the world frame.

<img src="https://upkie.github.io/upkie/observers.png" align="right">

### Floor and wheel contact

The [FloorContact](\ref upkie::cpp::observers::FloorContact) observer detects contact between the wheels and the floor. The PID balancer used for testing relies on this observer, for instance, to reset its integrator while the robot is in the air, to avoid fast-spinning wheels at touchdown.

Internally, the floor contact observer relies on two [WheelContact](\ref upkie::cpp::observers::WheelContact) observers, one for each wheel.

### Wheel odometry

| Observation key | Description |
|-----------------|-------------|
| `wheel_odometry.position` | Ground position in [m] |
| `wheel_odometry.velocity` | Ground velocity in [m] / [s] |

This observer measures the relative motion of the robot with respect to the floor, outputting ground position and velocity estimates. These quantities are used for instance in the PID balancer, which tries to stay around the same spot on the floor. Check out the [WheelOdometry](\ref upkie::cpp::observers::WheelOdometry) observer API for details.

### History observer {#history-observer}

The history observer reports higher-frequency signals from the spine as vectors of observations to lower-frequency agents. It handles floating-point and vector-valued observations. For instance, to report the left-knee torque readings on an Upkie, create a shared-pointer as follows and add it to the observer pipeline of the spine:

```cpp
auto left_knee_torque_history = std::make_shared<HistoryObserver<double> >(
    /* keys = */ std::vector<std::string>{"servo", "left_knee", "torque"},
    /* size = */ 10,
    /* default_value = */ std::numeric_limits<double>::quiet_NaN());
observer_pipeline.append_observer(left_knee_torque_history);
```

The same syntax can be used for a vector like the IMU linear acceleration:

```cpp
auto linear_acceleration_history =
    std::make_shared<HistoryObserver<Eigen::Vector3d> >(
        /* keys = */ std::vector<std::string>{"imu", "linear_acceleration"},
        /* size = */ 10,
        /* default_value = */ Eigen::Vector3d::Zero());
observer_pipeline.append_observer(linear_acceleration_history);
```

Check out the [HistoryObserver](\ref upkie::cpp::observers::HistoryObserver) API reference for details.

## Simulation

Simulation spines may report additional observations:

### Floating base

| Observation key | Description |
|-----------------|-------------|
| `sim.base` | Positions and velocities of the base frame |
| `sim.base.position` | Position of the base frame in the world frame, in [m] |
| `sim.base.orientation` | Unit quaternion (q, x, y, z) of the orientation of the base frame in the world frame |
| `sim.base.linear_velocity` | Linear velocity from base to world in world, in [m] / [s] |
| `sim.base.angular_velocity` | Angular velocity from base to world in world, in [rad] / [s] |

### IMU

| Observation key | Description |
|-----------------|-------------|
| `sim.imu` | Non-observables related to the IMU |
| `sim.imu.linear_velocity` | Linear velocity of the IMU frame in the world frame, in [m] / [s] |

### Rigid bodies

| Observation key | Description |
|-----------------|-------------|
| `sim.bodies` | Coordinates of rigid bodies in the world frame |
| `sim.bodies.YYY` | Positions and velocities of the extra body YYY |
| `sim.bodies.YYY.position` | Position of YYY in the world frame, in [m] |
| `sim.bodies.YYY.orientation` | Unit quaternion (q, x, y, z) of the orientation of YYY in the world frame |
