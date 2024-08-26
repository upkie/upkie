# Observations {#observations}

Spines compute observation dictionaries from [sensor measurements](\ref sensors) by applying *observers* one after the other in a sequence called the *observer pipeline*. This page lists the outputs of available observers, using shorthands ``a.b.c`` for nested dictionary keys ``observation["a"]["b"]["c"]``.

## IMU observations

| Observation key | Description |
|-----------------|-------------|
| `imu.angular_velocity` | Body angular velocity of the IMU frame in [rad] / [s] |
| `imu.linear_acceleration` | Linear acceleration of the IMU, with gravity filtered out, in [m] / [s]² |
| `imu.orientation` | Unit quaternion (``qw``, ``qx``, ``qy``, ``qz``) of the orientation from the IMU frame to the [ARS](\ref ars) frame |
| `imu.raw_angular_velocity` | Raw measurement from the gyroscope of the IMU, in [rad] / [s] |
| `imu.raw_linear_acceleration` | [Proper acceleration](https://en.wikipedia.org/wiki/Accelerometer#Physical_principles) measured by the accelerometer of the IMU, in [m] / [s]² |

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

## Servo observations

| Observation key | Description |
|-----------------|-------------|
| `servo.X` | Observations for servo ``X`` in the servo layout |
| `servo.X.position` | Angle between the stator and the rotor in [rad] |
| `servo.X.torque` | Joint torque in [N m] |
| `servo.X.velocity` | Angular velocity of the rotor w.r.t. stator in rotor, in [rad] / [s] |

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
