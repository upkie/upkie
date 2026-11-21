# Observations {#observations}

[TOC]

Spines report observations to agents as nested dictionaries. This page uses shorthands `a.b.c` for nested dictionary keys `observation["a"]["b"]["c"]`.

At each cycle, a spine first reads measurements from its [sensors](\ref sensors), then computes estimates from these measurements by applying [observers](\ref observers) one after the other, in a sequence called the *observer pipeline*. Simulation spines also report [simulation-only](\ref sim-observations) quantities, such as the ground-truth state of the robot. \anchor time-observation Finally, the spine timestamps each observation with its `time` key, in seconds since the epoch.

## Sensors {#sensors}

Sensors provide raw measurements to the spine. IMU and servo measurements are always reported, as they come from the robot itself. Other sensors pull measurements from host devices and files, and are only reported when enabled in the spine.

### IMU {#imu-observations}

- Sensor: always reported
- See also: [ImuData](\ref upkie::cpp::interfaces::ImuData)
- Spine observation key: `imu`

| Observation key | Description |
|-----------------|-------------|
| `angular_velocity` | [Body angular velocity](\ref upkie::cpp::interfaces::ImuData::angular_velocity_imu_in_imu) of the IMU frame in rad/s |
| `linear_acceleration` | [Linear acceleration](\ref upkie::cpp::interfaces::ImuData::linear_acceleration_imu_in_imu) of the IMU, with gravity filtered out, in m/s² |
| `orientation` | [Orientation of the IMU frame](\ref upkie::cpp::interfaces::ImuData::orientation_imu_in_ars) in the [ARS](\ref ars) frame as a unit quaternion (w, x, y, z) |
| `raw_angular_velocity` | [Raw angular velocity](\ref upkie::cpp::interfaces::ImuData::raw_angular_velocity) measured by the gyroscope of the IMU, in rad/s |
| `raw_linear_acceleration` | [Raw linear acceleration](\ref upkie::cpp::interfaces::ImuData::raw_linear_acceleration) measured by the accelerometer of the IMU, in m/s² |

The inertial measurement unit (IMU) mounted on the [pi3hat](https://mjbots.com/products/mjbots-pi3hat-r4-5) combines an accelerometer and a gyroscope. These raw measurements are converted onboard by an unscented Kalman filter (based on a standard quasi-static assumption) that outputs observed quantities with respect to an attitude reference system (ARS) frame.

There are three main frames to keep in mind when considering the IMU: the frame of the IMU sensor itself, the world frame, and the [attitude reference system](\ref ars) (ARS) frame. The world frame is an inertial frame of reference we refer to in various observers. It has an x-axis pointing forward, a y-axis pointing to the left and a z-axis vertical and pointing up (*i.e.*, against the gravity vector).

### Servo {#servo-observations}

- Sensor: always reported
- Spine observation key: `servo`

| Observation key            | Description |
|----------------------------|-------------|
| `<servo_name>`             | Observations for a given servo, *e.g.* `left_hip` |
| `<servo_name>.d_current`   | D-axis current in A |
| `<servo_name>.fault`       | Fault code reported by the moteus controller (zero when there is no fault) |
| `<servo_name>.mode`        | Current [mode](https://github.com/mjbots/moteus/blob/main/docs/reference.md#0x000---mode) of the moteus controller |
| `<servo_name>.position`    | Angle between the stator and the rotor in radians |
| `<servo_name>.q_current`   | Q-axis current in A |
| `<servo_name>.temperature` | Temperature of the moteus controller board in °C |
| `<servo_name>.torque`      | Joint torque in N⋅m |
| `<servo_name>.velocity`    | Angular velocity of the rotor w.r.t. stator in rotor, in rad/s |
| `<servo_name>.voltage`     | Input voltage of the moteus controller board in V |

Actuators on the robot are [qdd100 beta 3](https://mjbots.com/products/qdd100-beta-3) (hips and knees) and [mj5208 brushless motors](https://mjbots.com/products/mj5208) (wheels). All of them have [moteus](https://mjbots.com/products/moteus-r4-11) controller boards that provide the above measurements. They are estimated as follows:

- Joint angle, sensed by two orthogonal Hall-effect sensors at the back of the moteus controller board (each measuring the magnetic field in one direction; the output angle is then the arc-tangent of the ratio between these two values)
- Joint velocity, obtained by filtering joint angle measurements (not a direct measurement)
- Joint torque, estimated from sensed phase currents (with a model that includes [stator magnetic saturation](https://jpieper.com/2020/07/31/dealing-with-stator-magnetic-saturation/))

### Joystick {#joystick-sensor}

- Sensor: [Joystick](\ref upkie::cpp::sensors::Joystick)
- Spine observation key: `joystick`

| Observation key | Description |
|-----------------|-------------|
| `cross_button` | Bottom button (cross on PS4, A on Xbox) |
| `left_axis` | Left analog joystick position (x, y) |
| `left_button` | Left button (L1 on PS4, L on Xbox) |
| `left_trigger` | Left trigger position |
| `pad_axis` | Directional pad position (x, y) |
| `right_axis` | Right analog joystick position (x, y) |
| `right_button` | Right button (R1 on PS4, R on Xbox) |
| `right_trigger` | Right trigger position |
| `square_button` | Left button (square on PS4, Y on Xbox) |
| `triangle_button` | Top button (triangle on PS4, X on Xbox) |

### Keyboard {#keyboard-sensor}

- Sensor: [Keyboard](\ref upkie::cpp::sensors::Keyboard)
- Spine observation key: `keyboard`

| Observation key | Description |
|-----------------|-------------|
| `key_pressed` | Whether a keyboard key is currently pressed |
| `up` | Whether the Up key is pressed |
| `down` | Whether the Down key is pressed |
| `left` | Whether the Left key is pressed |
| `right` | Whether the Right key is pressed |
| `a` | Whether the A key is pressed |
| `d` | Whether the D key is pressed |
| `s` | Whether the S key is pressed |
| `w` | Whether the W key is pressed |
| `x` | Whether the X key is pressed |
| `unknown` | Whether an unknown key is pressed |

### CPU temperature {#cpu-temperature}

- Sensor: [CpuTemperature](\ref upkie::cpp::sensors::CpuTemperature)
- Spine observation key: `cpu_temperature`

The observation is directly the CPU temperature of the spine host, in degrees Celsius.

## Observers {#observers}

Observers compute estimates from sensor measurements, and possibly from the outputs of previous observers in the observer pipeline. For instance, the wheel odometry observer relies on the output of the floor contact observer.

<img src="https://upkie.github.io/upkie/observers.png" align="right">

### Base orientation {#base-orientation}

- Observer: [BaseOrientation](\ref upkie::cpp::observers::BaseOrientation)
- Spine observation key: `base_orientation`

The base orientation observer estimates the orientation of the floating base with respect to the world frame.

| Observation key          | Description |
|--------------------------|-------------|
| `angular_velocity`       | Body angular velocity vector of the base frame in rad/s |
| `pitch`                  | Pitch angle of the base frame relative to the world frame, in radians |
| `rotation_base_to_world` | Rotation matrix from the base frame to the world frame |

### Floor contact {#floor-contact}

- Observer: [FloorContact](\ref upkie::cpp::observers::FloorContact)
- Spine observation key: `floor_contact`

The floor contact observer detects contact between the wheels and the floor. The PID balancer used for testing relies on this observer, for instance, to reset its integrator while the robot is in the air, to avoid fast-spinning wheels at touchdown. Internally, it relies on two [WheelContact](\ref upkie::cpp::observers::WheelContact) observers, one for each wheel, whose outputs are reported under `<wheel>`, which is either `left_wheel` or `right_wheel`.

| Observation key            | Description |
|----------------------------|-------------|
| `contact`                  | Boolean contact state |
| `upper_leg_torque`         | Mean squared upper-leg torques in N⋅m |
| `<wheel>.abs_acceleration` | Low-pass filtered absolute wheel acceleration in rad/s² |
| `<wheel>.abs_torque`       | Low-pass filtered absolute wheel torque in N⋅m |
| `<wheel>.contact`          | Contact state of the wheel (boolean) |
| `<wheel>.inertia`          | Apparent inertia at the wheel, *i.e.* absolute torque divided by absolute acceleration, in kg⋅m² |

### Wheel odometry {#wheel-odometry}

- Observer: [WheelOdometry](\ref upkie::cpp::observers::WheelOdometry)
- Spine observation key: `wheel_odometry`

This observer measures the relative motion of the robot with respect to the floor, outputting ground position and velocity estimates. These quantities are used for instance in the PID balancer, which tries to stay around the same spot on the floor.

| Observation key | Description                  |
|-----------------|------------------------------|
| `position`      | Ground position in meters    |
| `velocity`      | Ground velocity in m/s       |

### History observer {#history-observer}

- Observer: [HistoryObserver](\ref upkie::cpp::observers::HistoryObserver)
- Spine observation key: `history` (optional)

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

The history is then reported under the same keys prefixed by `history`, for instance `history.servo.left_knee.torque` in the first example.

## Simulation {#sim-observations}

- Spine observation key: `sim`

Simulation spines report the following additional observations, which are not observable on the real robot.

### Floating base {#sim-floating-base}

- Spine observation key: `sim.base`

Positions and velocities of the base frame:

| Observation key    | Description |
|--------------------|-------------|
| `position`         | Position of the base frame in the world frame, in meters |
| `orientation`      | Unit quaternion (w, x, y, z) of the orientation of the base frame in the world frame |
| `linear_velocity`  | Linear velocity from base to world in world, in m/s |
| `angular_velocity` | Angular velocity from base to world in world, in rad/s |

### IMU non-observables {#sim-imu}

- Spine observation key: `sim.imu`

Non-observable quantities related to the IMU:

| Observation key   | Description |
|-------------------|-------------|
| `linear_velocity` | Linear velocity of the IMU frame in the world frame, in m/s |

### Rigid bodies {#sim-bodies}

- Spine observation key: `sim.bodies`

Coordinates of some rigid bodies in the world frame:

| Observation key   | Description |
|-------------------|-------------|
| `YYY`             | Positions and velocities of the extra body YYY |
| `YYY.position`    | Position of YYY in the world frame, in meters |
| `YYY.orientation` | Unit quaternion (w, x, y, z) of the orientation of YYY in the world frame |

## All observations {#all-observations}

The following table lists all observation keys that spines can report, along with the section that details them. Placeholders in angle brackets depend on the robot or spine configuration: `<servo_name>` is a servo name such as `left_hip`, `<body_name>` is the name of an extra body in the simulation, and `<keys>` are the keys of a signal reported by a [history observer](\ref history-observer). Observations from optional [sensors](\ref sensors) are only reported when they are enabled in the spine, and [simulation](\ref sim-observations) observations only by simulation spines.

| Key                                          | Source                                    |
|----------------------------------------------|-------------------------------------------|
| `base_orientation.angular_velocity`          | [Base orientation](\ref base-orientation) |
| `base_orientation.pitch`                     | [Base orientation](\ref base-orientation) |
| `base_orientation.rotation_base_to_world`    | [Base orientation](\ref base-orientation) |
| `cpu_temperature`                            | [CPU temperature](\ref cpu-temperature)   |
| `floor_contact.contact`                      | [Floor contact](\ref floor-contact)       |
| `floor_contact.left_wheel.abs_acceleration`  | [Floor contact](\ref floor-contact)       |
| `floor_contact.left_wheel.abs_torque`        | [Floor contact](\ref floor-contact)       |
| `floor_contact.left_wheel.contact`           | [Floor contact](\ref floor-contact)       |
| `floor_contact.left_wheel.inertia`           | [Floor contact](\ref floor-contact)       |
| `floor_contact.right_wheel.abs_acceleration` | [Floor contact](\ref floor-contact)       |
| `floor_contact.right_wheel.abs_torque`       | [Floor contact](\ref floor-contact)       |
| `floor_contact.right_wheel.contact`          | [Floor contact](\ref floor-contact)       |
| `floor_contact.right_wheel.inertia`          | [Floor contact](\ref floor-contact)       |
| `floor_contact.upper_leg_torque`             | [Floor contact](\ref floor-contact)       |
| `history.<keys>`                             | [History observer](\ref history-observer) |
| `imu.angular_velocity`                       | [IMU](\ref imu-observations)              |
| `imu.linear_acceleration`                    | [IMU](\ref imu-observations)              |
| `imu.orientation`                            | [IMU](\ref imu-observations)              |
| `imu.raw_angular_velocity`                   | [IMU](\ref imu-observations)              |
| `imu.raw_linear_acceleration`                | [IMU](\ref imu-observations)              |
| `joystick.cross_button`                      | [Joystick](\ref joystick-sensor)          |
| `joystick.left_axis`                         | [Joystick](\ref joystick-sensor)          |
| `joystick.left_button`                       | [Joystick](\ref joystick-sensor)          |
| `joystick.left_trigger`                      | [Joystick](\ref joystick-sensor)          |
| `joystick.pad_axis`                          | [Joystick](\ref joystick-sensor)          |
| `joystick.right_axis`                        | [Joystick](\ref joystick-sensor)          |
| `joystick.right_button`                      | [Joystick](\ref joystick-sensor)          |
| `joystick.right_trigger`                     | [Joystick](\ref joystick-sensor)          |
| `joystick.square_button`                     | [Joystick](\ref joystick-sensor)          |
| `joystick.triangle_button`                   | [Joystick](\ref joystick-sensor)          |
| `keyboard.a`                                 | [Keyboard](\ref keyboard-sensor)          |
| `keyboard.d`                                 | [Keyboard](\ref keyboard-sensor)          |
| `keyboard.down`                              | [Keyboard](\ref keyboard-sensor)          |
| `keyboard.key_pressed`                       | [Keyboard](\ref keyboard-sensor)          |
| `keyboard.left`                              | [Keyboard](\ref keyboard-sensor)          |
| `keyboard.right`                             | [Keyboard](\ref keyboard-sensor)          |
| `keyboard.s`                                 | [Keyboard](\ref keyboard-sensor)          |
| `keyboard.unknown`                           | [Keyboard](\ref keyboard-sensor)          |
| `keyboard.up`                                | [Keyboard](\ref keyboard-sensor)          |
| `keyboard.w`                                 | [Keyboard](\ref keyboard-sensor)          |
| `keyboard.x`                                 | [Keyboard](\ref keyboard-sensor)          |
| `servo.<servo_name>.d_current`               | [Servo](\ref servo-observations)          |
| `servo.<servo_name>.fault`                   | [Servo](\ref servo-observations)          |
| `servo.<servo_name>.mode`                    | [Servo](\ref servo-observations)          |
| `servo.<servo_name>.position`                | [Servo](\ref servo-observations)          |
| `servo.<servo_name>.q_current`               | [Servo](\ref servo-observations)          |
| `servo.<servo_name>.temperature`             | [Servo](\ref servo-observations)          |
| `servo.<servo_name>.torque`                  | [Servo](\ref servo-observations)          |
| `servo.<servo_name>.velocity`                | [Servo](\ref servo-observations)          |
| `servo.<servo_name>.voltage`                 | [Servo](\ref servo-observations)          |
| `sim.base.angular_velocity`                  | [Floating base](\ref sim-floating-base)   |
| `sim.base.linear_velocity`                   | [Floating base](\ref sim-floating-base)   |
| `sim.base.orientation`                       | [Floating base](\ref sim-floating-base)   |
| `sim.base.position`                          | [Floating base](\ref sim-floating-base)   |
| `sim.bodies.<body_name>.orientation`         | [Rigid bodies](\ref sim-bodies)           |
| `sim.bodies.<body_name>.position`            | [Rigid bodies](\ref sim-bodies)           |
| `sim.imu.linear_velocity`                    | [IMU non-observables](\ref sim-imu)       |
| `time`                                       | [Timestamp](\ref time-observation)        |
| `wheel_odometry.position`                    | [Wheel odometry](\ref wheel-odometry)     |
| `wheel_odometry.velocity`                    | [Wheel odometry](\ref wheel-odometry)     |
