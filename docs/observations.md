# Observations {#observations}

Here is an index of observation dictionaries. Keys are a shorthand for nested dictionaries where ``a.b.c`` corresponds to ``observation["a"]["b"]["c"]``.

| Observation key | Description |
|-----------------|-------------|
| `base_orientation.angular_velocity` | Body angular velocity of the base frame in [rad] / [s] |
| `base_orientation.pitch` | Pitch angle of the base frame relative to the world frame, in radians |
| `imu` | Inertial measurement unit, see [ImuData](\ref upkie::cpp::actuation::ImuData) for more details |
| `imu.angular_velocity` | Body angular velocity of the IMU frame in [rad] / [s] |
| `imu.linear_acceleration` | Linear acceleration of the IMU, with gravity filtered out, in [m] / [s]² |
| `imu.orientation` | Unit quaternion (``qw``, ``qx``, ``qy``, ``qz``) of the orientation from the IMU frame to the [ARS](\ref ars) frame |
| `imu.raw_angular_velocity` | Raw value read from the gyroscope of the IMU, in [rad] / [s] |
| `imu.raw_linear_acceleration` | [Proper acceleration](https://en.wikipedia.org/wiki/Accelerometer#Physical_principles) measured by the accelerometer of the IMU, in [m] / [s]² |
| `servos` | Servo motor measurements |
| `servos.X` | Observations for servo ``X`` in the servo layout |
| `servos.X.position` | Angle between the stator and the rotor in [rad] |
| `servos.X.torque` | Joint torque in [N] * [m] |
| `servos.X.velocity` | Angular velocity of the rotor w.r.t. stator in rotor, in [rad] / [s] |
| `wheel_odometry` | Wheel odometry, , see [WheelOdometry](\ref upkie::cpp::observers::WheelOdometry) for more details |
| `wheel_odometry.position` | Ground position in [m] |
| `wheel_odometry.velocity` | Ground velocity in [m] / [s] |

See also [Sensors](\ref sensors).

## Attitude reference system {#ars}

The attitude reference system (ARS) frame that the IMU filter maps to has its x-axis pointing forward, y-axis pointing to the right and z-axis pointing down ([details](https://github.com/mjbots/pi3hat/blob/ab632c82bd501b9fcb6f8200df0551989292b7a1/docs/reference.md#orientation)). This is not the convention we use in the world frame, where the x-axis points forward, the y-axis points left and the z-axis points up. The rotation matrix from the ARS frame to the world frame is thus:

\f$
R_{WA} = \begin{bmatrix}
    1 & 0 & 0 \\
    0 & -1 & 0 \\
    0 & 0 & -1 \\
\end{bmatrix}
\f$

## Observers {#observers}

<img src="https://upkie.github.io/upkie/observers.png" align="right">

<dl>
  <dt><a href="https://upkie.github.io/upkie/classupkie_1_1observers_1_1FloorContact.html#details">Floor contact</a></dt>
  <dd>Detect contact between the wheels and the floor. The pink and wheel balancers use contact as a reset flag for their integrators, to avoid over-spinning the wheels while the robot is in the air.</dd>

  <dt><a href="https://upkie.github.io/upkie/classupkie_1_1observers_1_1WheelContact.html#details">Wheel contact</a></dt>
  <dd>Detect contact between a given wheel and the floor.</dd>

  <dt><a href="https://upkie.github.io/upkie/classupkie_1_1observers_1_1WheelOdometry.html#details">Wheel odometry</a></dt>
  <dd>Measure the relative motion of the floating base with respect to the floor. Wheel odometry is part of their secondary task (after keeping the head straight), which is to stay around the same spot on the floor.</dd>
</dl>

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

Check out the API reference for details: \ref upkie::cpp::observers::HistoryObserver.
