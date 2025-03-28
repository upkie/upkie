# Gym environments {#gym-environments}

Upkie has environments compatible with the [Gymnasium API](https://gymnasium.farama.org/):

- [UpkieServos](\ref upkie_base_env_description): base environment where actions command servomotors directly.
    - [UpkieGroundVelocity](\ref upkie_ground_velocity_description): behave like a wheeled inverted pendulum.
    - [UpkieServoPositions](\ref upkie_servo_positions_description): joint position control only.
    - [UpkieServoTorques](\ref upkie_servo_torques_description): joint torque control only.

While each environment has its own observation and action spaces, all of them also report full [spine observations](\ref observations) in their `info` dictionaries.
