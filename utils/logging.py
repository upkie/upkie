#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2022 Stéphane Caron
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

"""
Log binary data to disk while running an agent.
"""

import asyncio

import aiofiles
import msgpack
from vulp.utils.serialize import serialize


async def write_log(path: str, logging_queue: asyncio.Queue) -> None:
    """
    Write messages from a logging queue to a file.

    Args:
        path: Path to output file.
        logging_queue: Queue with input logging messages.
    """
    logger = await aiofiles.open(path, "wb")
    packer = msgpack.Packer(default=serialize, use_bin_type=True)
    while True:
        log_message = await logging_queue.get()
        if log_message == {"exit": True}:
            break
        await logger.write(packer.pack(log_message))
        # Flushing has little effect when the Python process is configured on
        # its own core (CPUID). When running on the default core, it tends to
        # make the slack duration of the other (control) coroutine more
        # predictable although a little bit lower on average.
        await logger.flush()
    await logger.close()
