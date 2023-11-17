# Observations {#observations}

Here is an index to learn more about the data written to observation dictionaries:

| Key | Short description | Type/Unit | Details |
|-----|-------------------|-----------|---------|
| `imu` | Inertial measurement unit on the pi3hat | | |
| `imu/angular_velocity` | Body angular velocity of the IMU frame | [rad] / [s] | [`ImuData`](https://upkie.github.io/vulp/structvulp_1_1actuation_1_1ImuData.html) |
| `imu/linear_acceleration` | Body linear acceleration of the IMU | [m] / [s]² | [`ImuData`](https://upkie.github.io/vulp/structvulp_1_1actuation_1_1ImuData.html) |
| `imu/orientation` | Orientation from the IMU frame to the attitude reference system (ARS) frame | Unit quat. | [`ImuData`](https://upkie.github.io/vulp/structvulp_1_1actuation_1_1ImuData.html) |
| `servos` | Servo motor measurements | | |
| `servos/<name>` | Name of the servo, defined in the servo layout | | @ref upkie::config::servo_layout |
| `servos/<name>/position` | Angle between the stator and the rotor | [rad] | |
| `servos/<name>/torque`   | Joint torque | [N] [m] | |
| `servos/<name>/velocity` | Angular velocity of the rotor w.r.t. stator in rotor | [rad] / [s] | |

See also [Sensors](@ref sensors).
