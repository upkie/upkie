# Environments {#environments}

Upkie has environments compatible with the [Gymnasium API](https://gymnasium.farama.org/):

- [UpkieBaseEnv](\ref upkie.envs.upkie_base_env.UpkieBaseEnv): base class for all Upkie environments.
    - [UpkieGroundVelocity](\ref upkie_ground_velocity_description): behave like a wheeled inverted pendulum.
    - [UpkieServos](\ref upkie_servos_description): action and observation correspond to the full servo API.
        - [UpkieServoPositions](\ref upkie_servo_positions_description): joint position control only.
        - [UpkieServoTorques](\ref upkie_servo_torques_description): joint torque control only.

While each environment has its own observation and action spaces, all of them also report full [spine observations](\ref observations) in their `info` dictionaries.
