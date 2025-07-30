# Gym environments {#gym-environments}

Upkie has environments compatible with the [Gymnasium API](https://gymnasium.farama.org/):

- [UpkieGroundVelocity](\ref upkie_ground_velocity_description): behave like a wheeled inverted pendulum.
- [UpkieServos](\ref upkie_servo_pipeline): control joint servos directly with torque feedforward and position-velocity feedback.

While each environment has its own observation and action spaces, all of them also report full [spine observations](\ref observations) in their `info` dictionaries.
