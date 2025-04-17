#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

from typing import Optional, Tuple

import gymnasium as gym
import numpy as np
import upkie_description
from loop_rate_limiters import RateLimiter

from upkie.exceptions import UpkieException
from upkie.model import Model
from upkie.utils.robot_state import RobotState


class UpkieServos(gym.Env):
    r"""!
    Base Upkie environment where actions command servomotors directly.

    \anchor upkie_servos_description

    Actions and observations correspond to the moteus servo API. Under the
    hood, the environment provides a number of features:

    - Communication with the spine process.
    - Initial state randomization (e.g. when training a policy).
    - Loop frequency regulation (optional).

    Note that Upkie environments are made to run on a single CPU thread. The
    downside for reinforcement learning is that computations are not massively
    parallel. The upside is that it simplifies deployment to the real robot, as
    it relies on the same spine interface that runs on real robots.

    ### Action space

    The action space is a dictionary with one key for each servo:

    - `left_hip`: left hip joint (qdd100)
    - `left_knee`: left knee joint (qdd100)
    - `left_wheel`: left wheel joint (mj5208)
    - `right_hip`: right hip joint (qdd100)
    - `right_knee`: right knee joint (qdd100)
    - `right_wheel`: right wheel joint (mj5208)

    The value for each servo dictionary is itself a dictionary with the
    following keys:

    - `position`: commanded joint angle \f$\theta^*\f$ in [rad] (NaN to
       disable) (required).
    - `velocity`: commanded joint velocity \f$\dot{\theta}^*\f$ in [rad] /
       [s] (required).
    - `feedforward_torque`: feedforward joint torque \f$\tau_{\mathit{ff}}\f$
       in [N m].
    - `kp_scale`: scaling factor \f$k_{p}^{\mathit{scale}}\f$ applied to the
       position feedback gain, between zero and one.
    - `kd_scale`: scaling factor \f$k_{d}^{\mathit{scale}}\f$ applied to the
       velocity feedback gain, between zero and one.
    - `maximum_torque`: maximum joint torque \f$\tau_{\mathit{max}}\f$
       (feedforward + feedback) enforced during the whole actuation step, in
       [N m].

    The resulting torque applied by the servo is then:

    \f[
    \begin{align*}
    \tau & = \underset{
            [-\tau_{\mathit{max}}, +\tau_{\mathit{max}}]}{
            \mathrm{clamp}
        }
        \left(
            \tau_{\mathit{ff}} +
            k_{p} k_{p}^{\mathit{scale}} (\theta^* - \theta) +
            k_{d} k_{d}^{\mathit{scale}} (\dot{\theta}^* - \dot{\theta}))
        \right)
    \end{align*}
    \f]

    Position and velocity gains \f$k_{p}\f$ and \f$k_{d}\f$ are configured in
    each moteus controller directly and don't change during execution. We can
    rather modulate the overall feedback gains via the normalized parameters
    \f$k_{p}^{\mathit{scale}} \in [0, 1]\f$ and \f$k_{d}^{\mathit{scale}} \in
    [0, 1]\f$. Note that the servo regulates the torque above at its own
    frequency, which is higher (typically 40 kHz) than the agent and the spine
    frequencies. See the [moteus
    reference](https://github.com/mjbots/moteus/blob/13c171c697ce6f60a73c9385e6fe951957313d1d/docs/reference.md#theory-of-operation)
    for more details.

    ### Observation space

    The observation space is a dictionary with one key for each servo. The
    value for each key is a dictionary with keys:

    - `position`: Joint angle in [rad].
    - `velocity`: Joint velocity in [rad] / [s].
    - `torque`: Joint torque in [N m].
    - `temperature`: Servo temperature in degree Celsius.
    - `voltage`: Power bus voltage of the servo, in [V].

    Full observations from the spine (detailed in \ref observations) are also
    available in the `info` dictionary returned by the reset and step
    functions.
    """

    __frequency: Optional[float]
    __rate: Optional[RateLimiter]
    __regulate_frequency: bool

    ## \var action_space
    ## Action space.
    action_space: gym.spaces.dict.Dict

    ## \var init_state
    ## Initial state for the floating base of the robot, which may be
    ## randomized upon resets.
    init_state: RobotState

    ## \var model
    ## Robot model read from its URDF description.
    model: Model

    ## \var observation_space
    ## Observation space.
    observation_space: gym.spaces.dict.Dict

    def __init__(
        self,
        frequency: Optional[float] = 200.0,
        frequency_checks: bool = True,
        init_state: Optional[RobotState] = None,
        regulate_frequency: bool = True,
    ) -> None:
        r"""!
        Initialize environment.

        \param frequency Regulated frequency of the control loop, in Hz. Can be
            prescribed even when `regulate_frequency` is unset, in which case
            `self.dt` will be defined but the loop frequency will not be
            regulated.
        \param frequency_checks If `regulate_frequency` is set and this
            parameter is true (default), a warning is issued every time the
            control loop runs slower than the desired `frequency`. Set this
            parameter to false to disable these warnings.
        \param init_state Initial state of the robot, only used in simulation.
        \param regulate_frequency If set (default), the environment will
            regulate the control loop frequency to the value prescribed in
            `frequency`.

        \throw SpineError If the spine did not respond after the prescribed
            number of trials.
        """
        if regulate_frequency and frequency is None:
            raise UpkieException(f"{regulate_frequency=} but {frequency=}")
        if init_state is None:
            init_state = RobotState(
                position_base_in_world=np.array([0.0, 0.0, 0.6])
            )

        action_space = {}
        neutral_action = {}
        max_action = {}
        min_action = {}
        servo_space = {}

        model = Model(upkie_description.URDF_PATH)
        for joint in model.joints:
            action_space[joint.name] = gym.spaces.Dict(
                {
                    "position": gym.spaces.Box(
                        low=joint.limit.lower,
                        high=joint.limit.upper,
                        shape=(1,),
                        dtype=float,
                    ),
                    "velocity": gym.spaces.Box(
                        low=-joint.limit.velocity,
                        high=+joint.limit.velocity,
                        shape=(1,),
                        dtype=float,
                    ),
                    "feedforward_torque": gym.spaces.Box(
                        low=-joint.limit.effort,
                        high=+joint.limit.effort,
                        shape=(1,),
                        dtype=float,
                    ),
                    "kp_scale": gym.spaces.Box(
                        low=0.0,
                        high=1.0,
                        shape=(1,),
                        dtype=float,
                    ),
                    "kd_scale": gym.spaces.Box(
                        low=0.0,
                        high=1.0,
                        shape=(1,),
                        dtype=float,
                    ),
                    "maximum_torque": gym.spaces.Box(
                        low=0.0,
                        high=joint.limit.effort,
                        shape=(1,),
                        dtype=float,
                    ),
                }
            )
            servo_space[joint.name] = gym.spaces.Dict(
                {
                    "position": gym.spaces.Box(
                        low=joint.limit.lower,
                        high=joint.limit.upper,
                        shape=(1,),
                        dtype=float,
                    ),
                    "velocity": gym.spaces.Box(
                        low=-joint.limit.velocity,
                        high=+joint.limit.velocity,
                        shape=(1,),
                        dtype=float,
                    ),
                    "torque": gym.spaces.Box(
                        low=-joint.limit.effort,
                        high=+joint.limit.effort,
                        shape=(1,),
                        dtype=float,
                    ),
                    "temperature": gym.spaces.Box(
                        low=0.0,
                        high=100.0,
                        shape=(1,),
                        dtype=float,
                    ),
                    "voltage": gym.spaces.Box(
                        low=10.0,  # moteus min 10 V
                        high=44.0,  # moteus max 44 V
                        shape=(1,),
                        dtype=float,
                    ),
                }
            )
            neutral_action[joint.name] = {
                "position": np.nan,
                "velocity": 0.0,
                "feedforward_torque": 0.0,
                "kp_scale": 1.0,
                "kd_scale": 1.0,
                "maximum_torque": joint.limit.effort,
            }
            max_action[joint.name] = {
                "position": joint.limit.upper,
                "velocity": joint.limit.velocity,
                "feedforward_torque": joint.limit.effort,
                "kp_scale": 1.0,
                "kd_scale": 1.0,
                "maximum_torque": joint.limit.effort,
            }
            min_action[joint.name] = {
                "position": joint.limit.lower,
                "velocity": -joint.limit.velocity,
                "feedforward_torque": -joint.limit.effort,
                "kp_scale": 0.0,
                "kd_scale": 0.0,
                "maximum_torque": 0.0,
            }

        # gymnasium.Env: action_space
        self.action_space = gym.spaces.Dict(action_space)

        # gymnasium.Env: observation_space
        self.observation_space = gym.spaces.Dict(servo_space)

        # Class attributes
        self.__frequency = frequency
        self.__frequency_checks = frequency_checks
        self.__max_action = max_action
        self.__min_action = min_action
        self.__neutral_action = neutral_action
        self.__rate = None
        self.__regulate_frequency = regulate_frequency
        self.init_state = init_state
        self.model = model

    @property
    def dt(self) -> Optional[float]:
        """!
        Regulated period of the control loop in seconds, or `None` if there
        is no loop frequency regulation.
        """
        return 1.0 / self.__frequency if self.__frequency is not None else None

    @property
    def frequency(self) -> Optional[float]:
        """!
        Regulated frequency of the control loop in Hz, or `None` if there is
        no loop frequency regulation.
        """
        return self.__frequency

    def get_neutral_action(self) -> dict:
        r"""!
        Get the neutral action where servos don't move.

        \return Neutral action where servos don't move.
        """
        return self.__neutral_action.copy()

    def log(self, name: str, entry) -> None:
        r"""!
        Log a new entry to the spine, if one is connected.

        \param name Name of the entry.
        \param entry Dictionary to log along with the actual action.
        """

    def reset(
        self,
        seed: Optional[int] = None,
        options: Optional[dict] = None,
    ) -> Tuple[dict, dict]:
        r"""!
        Resets the spine and get an initial observation.

        \param seed Number used to initialize the environment’s internal random
            number generator.
        \param options Currently unused.
        \return
            - `observation`: Initial vectorized observation, i.e. an element
              of the environment's `observation_space`.
            - `info`: Dictionary with auxiliary diagnostic information. For
              Upkie this is the full observation dictionary sent by the spine.
        """
        super().reset(seed=seed)
        if self.__regulate_frequency:
            rate_name = f"{self.__class__.__name__} rate limiter"
            self.__rate = RateLimiter(
                self.__frequency,
                name=rate_name,
                warn=self.__frequency_checks,
            )
        return {}, {}

    def step(self) -> None:
        r"""!
        Regulate the control loop frequency, if applicable.
        """
        if self.__regulate_frequency:
            self.__rate.sleep()  # wait until clock tick to send the action
            self.log("rate", {"slack": self.__rate.slack})

    def update_init_rand(self, **kwargs) -> None:
        r"""!
        Update initial-state randomization.

        Keyword arguments are forwarded as is to \ref
        upkie.utils.robot_state_randomization.RobotStateRandomization.update.
        """
        self.init_state.randomization.update(**kwargs)
