# Sensors

## Inertial measurement unit

The inertial measurement unit (IMU) mounted on the [pi3hat](https://mjbots.com/products/mjbots-pi3hat-r4-5) produces the following measurements with respect to the attitude reference system (ARS) frame:

- Linear acceleration of the IMU frame in the IMU frame
- Body angular velocity of the IMU frame (that is, from the IMU frame to the ARS frame, expressed in the IMU frame)
- Orientation of the IMU frame in the ARS frame

These measurements are exposed by the spine in ``observation["imu"]``.

## mjbots actuators

All mjbots actuators on the robot, be it qdd100's in the hips and knees or mj5208's in the wheels, have [moteus](https://mjbots.com/products/moteus-r4-11) controller boards that provide the following measurements:

- Joint angle, sensed by two orthogonal Hall-effect sensors at the back of the moteus controller board (each measuring the magnetic field in one direction; the output angle is then the arc-tangent of the ratio between these two values)
- Joint velocity, obtained by filtering joint angle measurements (not a direct measurement)
- Joint torque, estimated from sensed phase currents (with a model that includes [stator magnetic saturation](https://jpieper.com/2020/07/31/dealing-with-stator-magnetic-saturation/)

These measurements are exposed by the spine ni ``observation["servos"]``.
