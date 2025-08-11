# Gym environments {#gym-environments}

[TOC]

Upkie has environments compatible with the [Gymnasium API](https://gymnasium.farama.org/):

| Environment ID            | Interface | Action and observation spaces |
|---------------------------|-----------|-------------------|
| `Upkie-Mock-Pendulum`     | Mock      | [Pendulum](\ref upkie_pendulum_description) |
| `Upkie-Mock-Servos`       | Mock      | [Servos](\ref upkie_servos_description) |
| `Upkie-Spine-Pendulum`    | Spine     | [Pendulum](\ref upkie_pendulum_description) |
| `Upkie-Spine-Servos`      | Spine     | [Servos](\ref upkie_servos_description) |
| `Upkie-PyBullet-Pendulum` | PyBullet  | [Pendulum](\ref upkie_pendulum_description) |
| `Upkie-PyBullet-Servos`   | PyBullet  | [Servos](\ref upkie_servos_description) |

While each environment has its own observation and action spaces, all of them also report full [spine observations](\ref observations) in their `info` dictionaries.
