# Observations {#observations}

The following index gives pointers to understand the data in observation dictionaries.

| Key | Short description | Type/Unit | Details |
|-----|-------------------|-----------|---------|
| `imu` | Inertial measurement unit on the pi3hat | | |
| `├── orientation` | Orientation from the IMU frame to the attitude reference system (ARS) frame | Unit quat. | [`ImuData`](https://tasts-robots.org/doc/vulp/structvulp_1_1actuation_1_1ImuData.html) |
| `├── linear_acceleration` | Body linear acceleration of the IMU | [m] / [s]² | [`ImuData`](https://tasts-robots.org/doc/vulp/structvulp_1_1actuation_1_1ImuData.html) |
| `└── angular_velocity` | Body angular velocity of the IMU frame | [rad] / [s] | [`ImuData`](https://tasts-robots.org/doc/vulp/structvulp_1_1actuation_1_1ImuData.html) |
| `servos` | Servo motor measurements | | |
| `└── <servo_name>` | Name of the servo, defined in the servo layout | | @ref upkie::config::servo_layout |
| `___ ├── position` | Angle between the stator and the rotor | [rad] | |
| `___ └── torque`   | Joint torque | [N] [m] | |
| `___ └── velocity` | Angular velocity of the rotor w.r.t. stator in rotor | [rad] / [s] | |

See also [Sensors](@ref sensors).
