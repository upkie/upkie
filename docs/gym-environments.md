# Gym environments {#gym-environments}

Upkie has environments compatible with the [Gymnasium API](https://gymnasium.farama.org/):

- [UpkieGroundVelocity](\ref upkie_ground_velocity_description): behave like a wheeled inverted pendulum.
- [UpkieServoPositions](\ref upkie_servo_positions_description): joint position control.
- [UpkieServoTorques](\ref upkie_servo_torques_description): joint torque control.
- [UpkieServos](\ref upkie_servos_description): control joint servos directly, including feedforward torques, position and velocity feedback.

While each environment has its own observation and action spaces, all of them also report full [spine observations](\ref observations) in their `info` dictionaries.
