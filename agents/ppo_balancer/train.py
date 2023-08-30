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
from typing import List

import gin
import gymnasium as gym
import stable_baselines3
import yaml
from gymnasium.wrappers.time_limit import TimeLimit
from rules_python.python.runfiles import runfiles
from settings import PPOSettings, Settings
from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback
from stable_baselines3.common.logger import TensorBoardOutputFormat
from standing_reward import StandingReward
from torch import nn

import upkie.envs
from upkie.envs import UpkieGroundVelocity
from upkie.utils.spdlog import logging
from utils import gin_operative_config_dict

upkie.envs.register()


class SummaryWriterCallback(BaseCallback):
    def __init__(self, env: UpkieGroundVelocity):
        super().__init__()
        self.env = env

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
        config = {
            "env": f"UpkieGroundVelocity-v{UpkieGroundVelocity.version}",
            "gin": gin_operative_config_dict(gin.config._OPERATIVE_CONFIG),
            "reward": self.env.reward.__dict__,
            "settings": Settings().__dict__,
            "spine_config": self.env.spine_config,
        }
        self.tb_formatter.writer.add_text(
            "config",
            f"```yaml\n{yaml.dump(config, indent=4)}\n```",
            global_step=None,
        )
        self.tb_formatter.writer.add_text(
            "gin_config",
            f"    {gin.operative_config_str()}".replace("\n", "\n    "),
            global_step=None,
        )


def train_policy(agent_name: str, training_dir: str) -> None:
    """
    Train a new policy and save it to a directory.

    Args:
        agent_name: Agent name.
        training_dir: Directory for logging and saving policies.
    """
    settings = Settings()
    agent_frequency = settings.agent_frequency
    max_episode_duration = settings.max_episode_duration
    policy_kwargs = {
        "activation_fn": nn.Tanh,
        "net_arch": [dict(pi=[64, 64], vf=[64, 64])],
    }
    env = TimeLimit(
        gym.make(
            "UpkieGroundVelocity-v1",
            frequency=agent_frequency,
            regulate_frequency=False,
            reward=StandingReward(),
            shm_name=f"/{agent_name}",
        ),
        max_episode_steps=int(max_episode_duration * agent_frequency),
    )

    dt = 1.0 / agent_frequency
    gamma = 1.0 - dt / settings.effective_time_horizon
    ppo_settings = PPOSettings()
    policy = stable_baselines3.PPO(
        "MlpPolicy",
        env,
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

    tb_log_name = f"{agent_name}_env-v{env.version}"
    try:
        policy.learn(
            total_timesteps=settings.total_timesteps,
            callback=[
                CheckpointCallback(
                    save_freq=int(1e5),
                    save_path=f"{training_dir}/{tb_log_name}_1",
                    name_prefix="checkpoint",
                ),
                SummaryWriterCallback(env),
            ],
            tb_log_name=tb_log_name,
        )
    except KeyboardInterrupt:
        logging.info("Training interrupted.")

    # Save policy no matter what!
    policy.save(f"{training_dir}/{agent_name}")
    policy.env.close()


def generate_agent_name():
    with open("/usr/share/dict/words") as fh:
        words = fh.read().splitlines()
    word_index = random.randint(0, len(words))
    while not words[word_index].isalnum():
        word_index = (word_index + 1) % len(words)
    return words[word_index]


def get_bullet_argv(agent_name: str, show: bool) -> List[str]:
    """
    Get command-line arguments for the Bullet spine.

    Args:
        agent_name: Agent name.
        show: If true, show simulator GUI.

    Returns:
        Command-line arguments.
    """
    settings = Settings()
    agent_frequency = settings.agent_frequency
    spine_frequency = settings.spine_frequency
    assert spine_frequency % agent_frequency == 0
    nb_substeps = spine_frequency / agent_frequency
    bullet_argv = []
    bullet_argv.extend(["--shm-name", f"/{agent_name}"])
    bullet_argv.extend(["--nb-substeps", str(nb_substeps)])
    bullet_argv.extend(["--spine-frequency", str(spine_frequency)])
    if show:
        bullet_argv.append("--show")
    return bullet_argv


if __name__ == "__main__":
    agent_dir = os.path.dirname(__file__)
    gin.parse_config_file(f"{agent_dir}/config.gin")

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--show",
        default=False,
        action="store_true",
        help="show simulator during trajectory rollouts",
    )
    args = parser.parse_args()

    agent_dir = os.path.dirname(__file__)
    agent_name = generate_agent_name()
    logging.info('New agent name is "%s"', agent_name)
    pid = os.fork()
    if pid == 0:  # child process: spine
        deez_runfiles = runfiles.Create()
        spine_path = os.path.join(
            agent_dir,
            deez_runfiles.Rlocation("upkie/spines/bullet"),
        )
        argv = get_bullet_argv(agent_name, show=args.show)
        os.execvp(spine_path, ["bullet"] + argv)
    else:  # parent process: trainer
        try:
            training_dir = f"{agent_dir}/policies"
            logging.info("Logging to %s", training_dir)
            logging.info(
                "To track in TensorBoard:\n\n\t"
                f"tensorboard --logdir {training_dir}"
                "\n\n"
            )
            train_policy(agent_name, training_dir)
        finally:
            os.kill(pid, signal.SIGINT)  # interrupt spine child process
            os.waitpid(pid, 0)  # wait for spine to terminate
