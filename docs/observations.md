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

