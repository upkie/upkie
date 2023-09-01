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

import datetime
import os


def get_log_path() -> str:
    """!
    Get `UPKIE_LOG_PATH` environment variable.

    @returns Log path.
    """
    return os.environ.get("UPKIE_LOG_PATH", "/tmp")


def new_log_filename(label: str):
    """!
    Get path to a new log, in the log path, with a given label.

    @params label Suffix of the new filename. (Prefix will be the date.)
    @returns Path to the new log file.
    """
    now = datetime.datetime.now()
    stamp = now.strftime("%Y-%m-%d_%H%M%S")
    log_path = get_log_path()
    return os.path.expanduser(f"{log_path}/{stamp}_{label}.mpack")
