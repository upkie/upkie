#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

from upkie.envs.backends import (
    GenesisBackend,
    MockBackend,
    PyBulletBackend,
    SpineBackend,
)
from upkie.envs.upkie_pendulum import UpkiePendulum
from upkie.exceptions import MissingOptionalDependency
from upkie.model import Model

from .upkie_servos import UpkieServos


def make_genesis_servos_env(**kwargs):
    r"""!
    Create an Upkie servos environment with Genesis backend.

    This function is meant to be called by `gymnasium.make()` rather than to be
    called directly.

    \param kwargs Keyword arguments forwarded to the environment and backend.
    \return UpkieServos with GenesisBackend.
    """
    backend_keys = {"genesis_init", "gui", "substeps"}
    backend_kwargs = {
        key: value for key, value in kwargs.items() if key in backend_keys
    }
    env_kwargs = {
        key: value for key, value in kwargs.items() if key not in backend_keys
    }

    # Create backend with appropriate dt from frequency
    frequency = env_kwargs.get("frequency", 200.0)
    dt = 1.0 / frequency if frequency is not None else 0.005
    backend = GenesisBackend(dt=dt, **backend_kwargs)

    return UpkieServos(backend=backend, **env_kwargs)


def make_mock_servos_env(**kwargs):
    r"""!
    Create an Upkie servos environment with Mock backend.

    This function is meant to be called by `gymnasium.make()` rather than to be
    called directly.

    \param kwargs Keyword arguments forwarded to the environment.
    \return UpkieServos with MockBackend.
    """
    backend = MockBackend()
    return UpkieServos(backend=backend, **kwargs)


def make_pybullet_servos_env(**kwargs):
    r"""!
    Create an Upkie servos environment with PyBullet backend.

    This function is meant to be called by `gymnasium.make()` rather than to be
    called directly.

    \param kwargs Keyword arguments forwarded to the environment and backend.
    \return UpkieServos with PyBulletBackend.
    """
    backend_keys = {"gui", "bullet_config", "nb_substeps"}
    backend_kwargs = {
        key: value for key, value in kwargs.items() if key in backend_keys
    }
    env_kwargs = {
        key: value for key, value in kwargs.items() if key not in backend_keys
    }

    # Create backend with appropriate dt from frequency
    frequency = env_kwargs.get("frequency", 200.0)
    dt = 1.0 / frequency if frequency is not None else 0.005
    backend = PyBulletBackend(dt=dt, **backend_kwargs)

    return UpkieServos(backend=backend, **env_kwargs)


def make_spine_servos_env(**kwargs):
    r"""!
    Create an Upkie servos environment with Spine backend.

    This function is meant to be called by `gymnasium.make()` rather than to be
    called directly.

    \param kwargs Keyword arguments forwarded to the environment and backend.
    \return UpkieServos with SpineBackend.
    """
    backend_keys = {"shm_name", "spine_config"}
    backend_kwargs = {
        key: value for key, value in kwargs.items() if key in backend_keys
    }
    env_kwargs = {
        key: value for key, value in kwargs.items() if key not in backend_keys
    }

    backend = SpineBackend(**backend_kwargs)
    return UpkieServos(backend=backend, **env_kwargs)


def wrap_pendulum(make_env_func, **kwargs):
    r"""!
    Make a pendulum environment around a servo environment.

    This function is meant to be called by `gymnasium.make()` rather than to be
    called directly.

    \param make_env_func Function that creates the base servo environment.
    \param kwargs Keyword arguments forwarded to both the
        \ref upkie.envs.upkie_pendulum.UpkiePendulum wrapper and the internal
        Upkie environment.
    \return UpkiePendulum environment.
    """
    pendulum_keys = {
        "fall_pitch",
        "left_wheeled",
        "max_ground_velocity",
    }
    pendulum_kwargs = {
        key: value for key, value in kwargs.items() if key in pendulum_keys
    }
    env_kwargs = {
        key: value for key, value in kwargs.items() if key not in pendulum_keys
    }
    env = make_env_func(**env_kwargs)
    return UpkiePendulum(env, **pendulum_kwargs)


def wrap_genesis_pendulum(**kwargs):
    r"""!
    Add pendulum wrapper around an UpkieServos with Genesis backend.
    """
    return wrap_pendulum(make_genesis_servos_env, **kwargs)


def wrap_mock_pendulum(**kwargs):
    r"""!
    Add pendulum wrapper around an UpkieServos with Mock backend.
    """
    return wrap_pendulum(make_mock_servos_env, **kwargs)


def wrap_pybullet_pendulum(**kwargs):
    r"""!
    Add pendulum wrapper around an UpkieServos with PyBullet backend.
    """
    return wrap_pendulum(make_pybullet_servos_env, **kwargs)


def wrap_spine_pendulum(**kwargs):
    r"""!
    Add pendulum wrapper around an UpkieServos with Spine backend.
    """
    return wrap_pendulum(make_spine_servos_env, **kwargs)


def make_cookie_pybullet_servos_env(**kwargs):
    r"""!
    Create a Cookie servos environment with PyBullet backend.

    This function is meant to be called by `gymnasium.make()` rather than to be
    called directly.

    \param kwargs Keyword arguments forwarded to the environment and backend.
    \return UpkieServos with PyBulletBackend and Cookie URDF.
    """
    try:
        import cookie_description
    except ImportError as e:
        raise MissingOptionalDependency(
            "cookie_description not found, "
            "install it via `pip install cookie_description`"
        ) from e
    cookie_model = Model(urdf_path=cookie_description.URDF_PATH)
    backend_keys = {"gui", "bullet_config", "nb_substeps"}
    backend_kwargs = {
        key: value for key, value in kwargs.items() if key in backend_keys
    }
    env_kwargs = {
        key: value for key, value in kwargs.items() if key not in backend_keys
    }
    frequency = env_kwargs.get("frequency", 200.0)
    dt = 1.0 / frequency if frequency is not None else 0.005
    backend = PyBulletBackend(dt=dt, model=cookie_model, **backend_kwargs)
    return UpkieServos(backend=backend, model=cookie_model, **env_kwargs)


def wrap_cookie_pybullet_pendulum(**kwargs):
    r"""!
    Add pendulum wrapper around an UpkieServos with PyBullet backend and Cookie
    URDF.
    """
    return wrap_pendulum(make_cookie_pybullet_servos_env, **kwargs)
