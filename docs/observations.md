# Observations

The following index gives pointers to understand the data in observation dictionaries.

| Key | Short description | Details |
|-----|-------------------|---------|
| `imu` | Inertial measurement unit on the pi3hat | |
| `├── orientation` | Orientation from the IMU frame to the attitude reference system (ARS) frame | [`ImuData`](https://tasts-robots.org/doc/vulp/structvulp_1_1actuation_1_1ImuData.html) |
| `├── linear_acceleration` | Body linear acceleration of the IMU in [m] / [s]² | [`ImuData`](https://tasts-robots.org/doc/vulp/structvulp_1_1actuation_1_1ImuData.html) |
| `└── angular_velocity` | Body angular velocity of the IMU frame in [rad] / [s] | [`ImuData`](https://tasts-robots.org/doc/vulp/structvulp_1_1actuation_1_1ImuData.html) |
