# Gym environments {#gym-environments}

[TOC]

Upkie has environments compatible with the [Gymnasium API](https://gymnasium.farama.org/):

| Environment ID            | Backend                                                                | Action and observation spaces               |
|---------------------------|------------------------------------------------------------------------|---------------------------------------------|
| `Upkie-Genesis-Pendulum`  | [Genesis](\ref upkie.envs.backends.genesis_backend.GenesisBackend)     | [Pendulum](\ref upkie_pendulum_description) |
| `Upkie-Genesis-Servos`    | [Genesis](\ref upkie.envs.backends.genesis_backend.GenesisBackend)     | [Servos](\ref upkie_servos_description)     |
| `Upkie-Mock-Pendulum`     | [Mock](\ref upkie.envs.backends.mock_backend.MockBackend)              | [Pendulum](\ref upkie_pendulum_description) |
| `Upkie-Mock-Servos`       | [Mock](\ref upkie.envs.backends.mock_backend.MockBackend)              | [Servos](\ref upkie_servos_description)     |
| `Upkie-PyBullet-Pendulum` | [PyBullet](\ref upkie.envs.backends.pybullet_backend.PyBulletBackend)  | [Pendulum](\ref upkie_pendulum_description) |
| `Upkie-PyBullet-Servos`   | [PyBullet](\ref upkie.envs.backends.pybullet_backend.PyBulletBackend)  | [Servos](\ref upkie_servos_description)     |
| `Upkie-Spine-Pendulum`    | [Spine](\ref upkie.envs.backends.spine_backend.SpineBackend)           | [Pendulum](\ref upkie_pendulum_description) |
| `Upkie-Spine-Servos`      | [Spine](\ref upkie.envs.backends.spine_backend.SpineBackend)           | [Servos](\ref upkie_servos_description)     |

While each environment has its own observation and action spaces, all of them also report full [spine observations](\ref observations) in their `info` dictionaries.
