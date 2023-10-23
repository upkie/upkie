#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria
# SPDX-License-Identifier: Apache-2.0

import argparse
import datetime
import os
import random
import signal
import tempfile
from typing import List

import gin
import gymnasium
import stable_baselines3
import yaml
from reward import Reward
from rules_python.python.runfiles import runfiles
from settings import EnvSettings, PPOSettings
from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback
from stable_baselines3.common.logger import TensorBoardOutputFormat
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.utils import set_random_seed
from stable_baselines3.common.vec_env import (
    DummyVecEnv,
    SubprocVecEnv,
    VecNormalize,
)
from stable_baselines3.common.vec_env.base_vec_env import VecEnv
from torch import nn
from utils import gin_operative_config_dict

import upkie.envs
from upkie.envs import InitRandomization
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
        reward = self.vec_env.get_attr("reward")[0]
        spine_config = self.vec_env.get_attr("spine_config")[0]
        config = {
            "env": EnvSettings().env_id,
            "gin": gin_operative_config_dict(gin.config._OPERATIVE_CONFIG),
            "reward": reward.__dict__,
            "spine_config": spine_config,
        }
        self.tb_formatter.writer.add_text(
            "config",
            f"```yaml\n{yaml.dump(config, indent=4)}\n```",
            global_step=None,
        )
        save_path = f"{self.save_path}/config.yaml"
        with open(save_path, "w") as fh:
            yaml.dump(config, fh, indent=4)
        logging.info(f"Saved configuration to {save_path}")


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
    settings = EnvSettings()
    agent_frequency = settings.agent_frequency
    spine_frequency = settings.spine_frequency
    assert spine_frequency % agent_frequency == 0
    nb_substeps = spine_frequency / agent_frequency
    bullet_argv = []
    bullet_argv.extend(["--shm-name", shm_name])
    bullet_argv.extend(["--nb-substeps", str(nb_substeps)])
    bullet_argv.extend(["--spine-frequency", str(spine_frequency)])
    if show:
        bullet_argv.append("--show")
    return bullet_argv


def make_env(
    spine_path: str,
    show: bool,
    subproc_index: int,
):
    settings = EnvSettings()
    seed = (
        settings.seed + subproc_index
        if settings.seed is not None
        else random.randint(0, 1_000_000)
    )

    def _init():
        shm_name = f"/{get_random_word()}"
        pid = os.fork()
        if pid == 0:  # child process: spine
            argv = get_bullet_argv(shm_name, show=show)
            os.execvp(spine_path, ["bullet"] + argv)
            return

        # parent process: trainer
        agent_frequency = settings.agent_frequency
        max_episode_duration = settings.max_episode_duration
        init_rand = InitRandomization(**settings.init_rand)
        env = gymnasium.make(
            settings.env_id,
            max_episode_steps=int(max_episode_duration * agent_frequency),
            # upkie.envs.UpkieBaseEnv
            frequency=agent_frequency,
            init_rand=init_rand,
            max_ground_accel=settings.max_ground_accel,
            max_ground_velocity=settings.max_ground_velocity,
            regulate_frequency=False,
            reward=Reward(),
            shm_name=shm_name,
            spine_config={
                "bullet": {
                    "torque_control": {
                        "kd": settings.sim_torque_control_kd,
                    },
                },
            },
            velocity_filter=settings.velocity_filter,
            velocity_filter_rand=settings.velocity_filter_rand,
        )
        env.reset(seed=seed)
        env._prepatch_close = env.close

        def close_monkeypatch():
            logging.info(f"Terminating spine {shm_name} with {pid=}...")
            os.kill(pid, signal.SIGINT)  # interrupt spine child process
            os.waitpid(pid, 0)  # wait for spine to terminate
            env._prepatch_close()

        env.close = close_monkeypatch
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

    deez_runfiles = runfiles.Create()
    spine_path = os.path.join(
        agent_dir,
        deez_runfiles.Rlocation("upkie/spines/bullet_spine"),
    )

    vec_env = (
        SubprocVecEnv(
            [
                make_env(spine_path, show, subproc_index=i)
                for i in range(nb_envs)
            ],
            start_method="fork",
        )
        if nb_envs > 1
        else DummyVecEnv([make_env(spine_path, show, subproc_index=0)])
    )
    if False:  # does not always improve returns during training
        vec_env = VecNormalize(vec_env)

    settings = EnvSettings()
    agent_frequency = settings.agent_frequency
    dt = 1.0 / agent_frequency
    gamma = 1.0 - dt / settings.discounted_horizon_duration

    ppo_settings = PPOSettings()
    policy = stable_baselines3.PPO(
        "MlpPolicy",
        vec_env,
        learning_rate=ppo_settings.learning_rate,
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
            "net_arch": dict(
                pi=ppo_settings.net_arch_pi,
                vf=ppo_settings.net_arch_vf,
            ),
        },
        verbose=1,
    )

    try:
        policy.learn(
            total_timesteps=settings.total_timesteps,
            callback=[
                CheckpointCallback(
                    save_freq=max(int(1e5) // nb_envs, 1_000),
                    save_path=save_path,
                    name_prefix="checkpoint",
                ),
                SummaryWriterCallback(vec_env, save_path),
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
    gin.parse_config_file(f"{agent_dir}/reward.gin")
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
