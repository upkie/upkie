#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import os
import random
import signal
import tempfile
from typing import List

import gin
import gymnasium as gym
import stable_baselines3
import yaml
from gymnasium.wrappers.time_limit import TimeLimit
from rules_python.python.runfiles import runfiles
from settings import EnvSettings, PPOSettings
from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback
from stable_baselines3.common.logger import TensorBoardOutputFormat
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import (
    DummyVecEnv,
    SubprocVecEnv,
    VecNormalize,
)
from stable_baselines3.common.vec_env.base_vec_env import VecEnv
from torch import nn
from utils import gin_operative_config_dict

import upkie.envs
from upkie.envs import SurvivalReward
from upkie.utils.spdlog import logging

upkie.envs.register()

running_spines = []


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
        "--nb-cpus",
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
    def __init__(self, vec_env: VecEnv, policy_name: str, training_dir: str):
        super().__init__()
        self.policy_name = policy_name
        self.training_dir = training_dir
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
        save_path = f"{self.training_dir}/{self.policy_name}.yaml"
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


def train_policy(
    policy_name: str,
    spine_path: str,
    training_dir: str,
    nb_cpus: int,
    show: bool,
) -> None:
    """!
    Train a new policy and save it to a directory.

    @param spine_path Path to the spine binary.
    @param training_dir Directory for logging and saving policies.
    @param show Whether to show the simulation GUI.
    """
    if policy_name == "":
        policy_name = get_random_word()
    logging.info('New policy name is "%s"', policy_name)

    settings = EnvSettings()
    agent_frequency = settings.agent_frequency
    max_episode_duration = settings.max_episode_duration
    policy_kwargs = {
        "activation_fn": nn.Tanh,
        "net_arch": dict(pi=[64, 64], vf=[64, 64]),
    }

    def make_env():
        shm_name = f"/{get_random_word()}"
        pid = os.fork()
        running_spines.append((shm_name, pid))
        if pid == 0:  # child process: spine
            argv = get_bullet_argv(shm_name, show=show)
            os.execvp(spine_path, ["bullet"] + argv)
            return

        # parent process: trainer
        env = gym.make(
            settings.env_id,
            frequency=agent_frequency,
            max_ground_accel=settings.max_ground_accel,
            max_ground_velocity=settings.max_ground_velocity,
            regulate_frequency=False,
            reward=SurvivalReward(),
            shm_name=shm_name,
        )

        return Monitor(  # monitor in TensorBoard
            TimeLimit(  # limit rollout durations
                env,
                max_episode_steps=int(max_episode_duration * agent_frequency),
            )
        )

    vec_env = (
        SubprocVecEnv([make_env for _ in range(nb_cpus)], start_method="fork")
        if nb_cpus > 1
        else DummyVecEnv([make_env])
    )
    if False:  # does not always improve returns during training
        vec_env = VecNormalize(vec_env)

    dt = 1.0 / agent_frequency
    gamma = 1.0 - dt / settings.cumulative_reward_horizon

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
        normalize_advantage=True,
        ent_coef=ppo_settings.ent_coef,
        vf_coef=ppo_settings.vf_coef,
        max_grad_norm=ppo_settings.max_grad_norm,
        use_sde=ppo_settings.use_sde,
        sde_sample_freq=ppo_settings.sde_sample_freq,
        target_kl=ppo_settings.target_kl,
        tensorboard_log=training_dir,
        policy_kwargs=policy_kwargs,
        verbose=1,
    )

    try:
        policy.learn(
            total_timesteps=settings.total_timesteps,
            callback=[
                CheckpointCallback(
                    save_freq=int(1e5),
                    save_path=f"{training_dir}/{policy_name}_1",
                    name_prefix="checkpoint",
                ),
                SummaryWriterCallback(vec_env, policy_name, training_dir),
            ],
            tb_log_name=policy_name,
        )
    except KeyboardInterrupt:
        logging.info("Training interrupted.")

    # Save policy no matter what!
    policy.save(f"{training_dir}/{policy_name}")
    policy.env.close()


if __name__ == "__main__":
    args = parse_command_line_arguments()
    agent_dir = os.path.dirname(__file__)
    gin.parse_config_file(f"{agent_dir}/settings.gin")

    deez_runfiles = runfiles.Create()
    spine_path = os.path.join(
        agent_dir,
        deez_runfiles.Rlocation("upkie/spines/bullet"),
    )

    try:
        training_dir = f"{tempfile.gettempdir()}/ppo_balancer"
        logging.info("Logging to %s", training_dir)
        logging.info(
            "To track in TensorBoard, run "
            f"`tensorboard --logdir {training_dir}`"
        )
        train_policy(
            args.name,
            spine_path,
            training_dir,
            nb_cpus=args.nb_cpus,
            show=args.show,
        )
    finally:
        for shm_name, pid in running_spines:
            logging.info(f"Terminating spine {shm_name} with {pid=}...")
            os.kill(pid, signal.SIGINT)  # interrupt spine child process
            os.waitpid(pid, 0)  # wait for spine to terminate
