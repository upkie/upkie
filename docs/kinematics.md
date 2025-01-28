# Kinematics {#kinematics}

This page is about the geometry and joint properties of Upkies. Mechanics and electronics are discussed in the [Hardware wiki](https://github.com/upkie/upkie/wiki).

## Joint limits {#joint-limits}

| Joint | Limit    | Value (±)  | Why? |
|-------|----------|------------|------|
| Hip   | Position | 1.26 rad   | Geometry: cables fully stretched |
| Hip   | Velocity | 28.8 rad/s | See qdd100 details below |
| Hip   | Torque   | 16.0 Nm    | Peak torque (< 1 s) from the [qdd100 spec](https://mjbots.com/products/qdd100-beta-3) |
| Knee  | Position | 2.51 rad   | Geometry: wheels touching hip actuators. |
| Knee  | Velocity | 28.8 rad/s | See qdd100 details below |
| Knee  | Torque   | 16   Nm    | Peak torque (< 1 s) from the [qdd100 spec](https://mjbots.com/products/qdd100-beta-3) |
| Wheel | Position | -          | No limit |
| Wheel | Velocity | 111  rad/s | See mj5208 details below |
| Wheel | Torque   | 1.7  Nm    | See mj5208 details below |

**For qdd100's:** peak velocities are [rated](https://mjbots.com/products/qdd100-beta-3) as 3,600 dps at 36 V and 2,300 dps at 24 V. Assuming a linear velocity-voltage relationship (which is a big assumption, for instance the actual maximum velocity will also depend on [`servo.pwm_rate_hz`](https://github.com/mjbots/moteus/blob/main/docs/reference.md#servopwm_rate_hz)) leads to 1,650 dps at the 18 V of the [RYOBI batteries used on Upkie](https://github.com/upkie/upkie/wiki/Bill-of-materials), or equivalently 28.8 rad/s after conversion and rounding.

**For mj5208's:** peak velocities from the [mj5208 spec](https://mjbots.com/products/mj5208) are rated as 7,500 rpm, or approximately 785 rad/s. Multiplying by the wheel radius of your typical Upkie (between 5 and 6 cm), we obtain a ground speed comparable to that of a car on a highway. We scale this down a notch! Backtracking from a top ground speed at 20 km/h, we obtain a maximum velocity around 111 rad/s.
