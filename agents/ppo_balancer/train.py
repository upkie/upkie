#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

import argparse
import datetime
import os
import random
import signal
import tempfile
from typing import Callable, List

import gin
import gymnasium
import numpy as np
import stable_baselines3
from envs import make_ppo_balancer_env
from rules_python.python.runfiles import runfiles
from settings import EnvSettings, PPOSettings, TrainingSettings
from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback
from stable_baselines3.common.logger import TensorBoardOutputFormat
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.utils import set_random_seed
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv
from stable_baselines3.common.vec_env.base_vec_env import VecEnv
from torch import nn

import upkie.envs
from upkie.utils.spdlog import logging

upkie.envs.register()


def parse_command_line_arguments() -> argparse.Namespace:
    """
    Parse command line arguments.

    Returns:
        Command-line arguments.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--name",
        default="",
        type=str,
        help="name of the new policy to train",
    )
    parser.add_argument(
        "--nb-envs",
        default=1,
        type=int,
        help="number of parallel simulation processes to run",
    )
    parser.add_argument(
        "--show",
        default=False,
        action="store_true",
        help="show simulator during trajectory rollouts",
    )
    return parser.parse_args()


class InitRandomizationCallback(BaseCallback):
    def __init__(
        self,
        vec_env: VecEnv,
        key: str,
        max_value: float,
        start_timestep: int,
        end_timestep: int,
    ):
        super().__init__()
        self.end_timestep = end_timestep
        self.key = key
        self.max_value = max_value
        self.start_timestep = start_timestep
        self.vec_env = vec_env

    def _on_step(self) -> bool:
        progress: float = np.clip(
            (self.num_timesteps - self.start_timestep) / self.end_timestep,
            0.0,
            1.0,
        )
        cur_value = progress * self.max_value
        self.vec_env.env_method("update_init_rand", **{self.key: cur_value})
        self.logger.record(f"init_rand/{self.key}", cur_value)


class SummaryWriterCallback(BaseCallback):
    def __init__(self, vec_env: VecEnv, save_path: str):
        super().__init__()
        self.save_path = save_path
        self.vec_env = vec_env

    def _on_training_start(self):
        output_formats = self.logger.output_formats
        self.tb_formatter = next(
            formatter
            for formatter in output_formats
            if isinstance(formatter, TensorBoardOutputFormat)
        )

    def _on_step(self) -> bool:
        # We wait for the first call to log operative config so that parameters
        # for functions called by the environment are logged as well.
        if self.n_calls != 1:
            return
        self.tb_formatter.writer.add_text(
            "gin/operative_config",
            gin.operative_config_str(),
            global_step=None,
        )
        gin_path = f"{self.save_path}/operative_config.gin"
        with open(gin_path, "w") as fh:
            fh.write(gin.operative_config_str())
        logging.info(f"Saved gin configuration to {gin_path}")


def get_random_word():
    with open("/usr/share/dict/words") as fh:
        words = fh.read().splitlines()
    word_index = random.randint(0, len(words))
    while not words[word_index].isalnum():
        word_index = (word_index + 1) % len(words)
    return words[word_index]


def get_bullet_argv(shm_name: str, show: bool) -> List[str]:
    """!
    Get command-line arguments for the Bullet spine.

    @param shm_name Name of the shared-memory file.
    @param show If true, show simulator GUI.
    @returns Command-line arguments.
    """
    env_settings = EnvSettings()
    agent_frequency = env_settings.agent_frequency
    spine_frequency = env_settings.spine_frequency
    assert spine_frequency % agent_frequency == 0
    nb_substeps = spine_frequency / agent_frequency
    bullet_argv = []
    bullet_argv.extend(["--shm-name", shm_name])
    bullet_argv.extend(["--nb-substeps", str(nb_substeps)])
    bullet_argv.extend(["--spine-frequency", str(spine_frequency)])
    if show:
        bullet_argv.append("--show")
    return bullet_argv


def init_env(
    max_episode_duration: float,
    show: bool,
    spine_path: str,
):
    """!
    Get an environment initialization function for a set of parameters.

    @param max_episode_duration Maximum duration of an episode, in seconds.
    @param show If true, show simulator GUI.
    @param spine_path Path to the Bullet spine binary.
    """
    env_settings = EnvSettings()
    seed = random.randint(0, 1_000_000)

    def _init():
        shm_name = f"/{get_random_word()}"
        pid = os.fork()
        if pid == 0:  # child process: spine
            argv = get_bullet_argv(shm_name, show=show)
            os.execvp(spine_path, ["bullet"] + argv)
            return

        # parent process: trainer
        agent_frequency = env_settings.agent_frequency
        velocity_env = gymnasium.make(
            env_settings.env_id,
            max_episode_steps=int(max_episode_duration * agent_frequency),
            frequency=agent_frequency,
            regulate_frequency=False,
            reward_weights=upkie.envs.UpkieGroundVelocity.RewardWeights(
                **env_settings.reward_weights
            ),
            shm_name=shm_name,
            spine_config=env_settings.spine_config,
            max_ground_velocity=env_settings.max_ground_velocity,
        )
        velocity_env.reset(seed=seed)
        velocity_env._prepatch_close = velocity_env.close

        def close_monkeypatch():
            logging.info(f"Terminating spine {shm_name} with {pid=}...")
            os.kill(pid, signal.SIGINT)  # interrupt spine child process
            os.waitpid(pid, 0)  # wait for spine to terminate
            velocity_env._prepatch_close()

        velocity_env.close = close_monkeypatch
        env = make_ppo_balancer_env(velocity_env, env_settings, training=True)
        return Monitor(env)

    set_random_seed(seed)
    return _init


def find_save_path(training_dir: str, policy_name: str):
    def path_for_iter(nb_iter: int):
        return f"{training_dir}/{policy_name}_{nb_iter}"

    nb_iter = 1
    while os.path.exists(path_for_iter(nb_iter)):
        nb_iter += 1
    return path_for_iter(nb_iter)


def affine_schedule(y_0: float, y_1: float) -> Callable[[float], float]:
    """!
    Affine schedule as a function over the [0, 1] interval.

    @param y_0 Function value at zero.
    @param y_1 Function value at one.
    @return Corresponding affine function.
    """
    diff = y_1 - y_0

    def schedule(x: float) -> float:
        """!
        Compute the current learning rate from remaining progress.

        @param x Progress decreasing from 1 (beginning) to 0.
        @return Corresponding learning rate>
        """
        return y_0 + x * diff

    return schedule


def train_policy(
    policy_name: str,
    training_dir: str,
    nb_envs: int,
    show: bool,
) -> None:
    """!
    Train a new policy and save it to a directory.

    @param policy_name Name of the trained policy.
    @param training_dir Directory for logging and saving policies.
    @param nb_envs Number of environments, each running in a separate process.
    @param show Whether to show the simulation GUI.
    """
    if policy_name == "":
        policy_name = get_random_word()
    save_path = find_save_path(training_dir, policy_name)
    logging.info('New policy name is "%s"', policy_name)
    logging.info("Training data will be logged to %s", save_path)

    training = TrainingSettings()
    deez_runfiles = runfiles.Create()
    spine_path = os.path.join(
        agent_dir,
        deez_runfiles.Rlocation("upkie/spines/bullet_spine"),
    )

    vec_env = (
        SubprocVecEnv(
            [
                init_env(
                    max_episode_duration=training.max_episode_duration,
                    show=show,
                    spine_path=spine_path,
                )
                for i in range(nb_envs)
            ],
            start_method="fork",
        )
        if nb_envs > 1
        else DummyVecEnv(
            [
                init_env(
                    max_episode_duration=training.max_episode_duration,
                    show=show,
                    spine_path=spine_path,
                )
            ]
        )
    )

    env_settings = EnvSettings()
    dt = 1.0 / env_settings.agent_frequency
    gamma = 1.0 - dt / training.return_horizon
    logging.info(
        "Discount factor gamma=%f for a return horizon of %f s",
        gamma,
        training.return_horizon,
    )

    ppo_settings = PPOSettings()
    policy = stable_baselines3.PPO(
        "MlpPolicy",
        vec_env,
        learning_rate=affine_schedule(
            y_1=ppo_settings.learning_rate,  # progress_remaining=1.0
            y_0=ppo_settings.learning_rate / 3,  # progress_remaining=0.0
        ),
        n_steps=ppo_settings.n_steps,
        batch_size=ppo_settings.batch_size,
        n_epochs=ppo_settings.n_epochs,
        gamma=gamma,
        gae_lambda=ppo_settings.gae_lambda,
        clip_range=ppo_settings.clip_range,
        clip_range_vf=ppo_settings.clip_range_vf,
        normalize_advantage=ppo_settings.normalize_advantage,
        ent_coef=ppo_settings.ent_coef,
        vf_coef=ppo_settings.vf_coef,
        max_grad_norm=ppo_settings.max_grad_norm,
        use_sde=ppo_settings.use_sde,
        sde_sample_freq=ppo_settings.sde_sample_freq,
        target_kl=ppo_settings.target_kl,
        tensorboard_log=training_dir,
        policy_kwargs={
            "activation_fn": nn.Tanh,
            "net_arch": {
                "pi": ppo_settings.net_arch_pi,
                "vf": ppo_settings.net_arch_vf,
            },
        },
        verbose=1,
    )

    try:
        policy.learn(
            total_timesteps=training.total_timesteps,
            callback=[
                CheckpointCallback(
                    save_freq=max(210_000 // nb_envs, 1_000),
                    save_path=save_path,
                    name_prefix="checkpoint",
                ),
                SummaryWriterCallback(vec_env, save_path),
                InitRandomizationCallback(
                    vec_env,
                    "pitch",
                    training.init_rand["pitch"],
                    start_timestep=0,
                    end_timestep=1e5,
                ),
                InitRandomizationCallback(
                    vec_env,
                    "v_x",
                    training.init_rand["v_x"],
                    start_timestep=0,
                    end_timestep=1e5,
                ),
                InitRandomizationCallback(
                    vec_env,
                    "omega_y",
                    training.init_rand["omega_y"],
                    start_timestep=0,
                    end_timestep=1e5,
                ),
            ],
            tb_log_name=policy_name,
        )
    except KeyboardInterrupt:
        logging.info("Training interrupted.")

    # Save policy no matter what!
    policy.save(f"{save_path}/final.zip")
    policy.env.close()


if __name__ == "__main__":
    args = parse_command_line_arguments()
    agent_dir = os.path.dirname(__file__)
    gin.parse_config_file(f"{agent_dir}/settings.gin")

    training_path = os.environ.get(
        "UPKIE_TRAINING_PATH", tempfile.gettempdir()
    )
    date = datetime.datetime.now().strftime("%Y-%m-%d")
    training_dir = f"{training_path}/{date}"
    logging.info("Logging training data in %s", training_dir)
    logging.info(
        "To track in TensorBoard, run "
        f"`tensorboard --logdir {training_dir}`"
    )
    train_policy(
        args.name,
        training_dir,
        nb_envs=args.nb_envs,
        show=args.show,
    )
