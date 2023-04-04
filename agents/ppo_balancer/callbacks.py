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

from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.logger import TensorBoardOutputFormat

import json
import gin
from upkie_locomotion.envs import UpkieWheelsEnv


class SummaryWriterCallback(BaseCallback):
    def __init__(self, env: UpkieWheelsEnv):
        super().__init__()
        self.config = env.config
        self.env = env

    def _on_training_start(self):
        output_formats = self.logger.output_formats
        self.tb_formatter = next(
            formatter
            for formatter in output_formats
            if isinstance(formatter, TensorBoardOutputFormat)
        )
        self.tb_formatter.writer.add_text(
            "env_id",
            f"UpkieWheelsEnv-v{UpkieWheelsEnv.version}",
            global_step=None,
        )
        self.tb_formatter.writer.add_text(
            "spine_config",
            f"```json\n{json.dumps(self.config, indent=4)}\n```",
            global_step=None,
        )

    def _on_step(self) -> bool:
        if self.n_calls == 1:
            # Wait for first call to log operative config so that parameters
            # for functions called by the environment are logged as well.
            self.tb_formatter.writer.add_text(
                "gin_config",
                f"```\n{gin.operative_config_str()}\n```",
                global_step=None,
            )
