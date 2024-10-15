#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron
# Copyright 2024 Inria

"""!
Inter-process communication interface.
"""

import mmap
import sys
from time import perf_counter_ns

import msgpack

from ..exceptions import PerformanceIssue, SpineError
from .request import Request
from .serialize import serialize
from .wait_for_shared_memory import wait_for_shared_memory


class SpineInterface:
    """!
    Interface to interact with a spine from a Python agent.
    """

    _mmap: mmap.mmap

    def __init__(
        self,
        shm_name: str = "/upkie",
        retries: int = 1,
        perf_checks: bool = True,
    ):
        r"""!
        Connect to the spine shared memory.

        \param shm_name Name of the shared memory object.
        \param retries Number of times to try opening the shared-memory file.
        \param perf_checks If true, run performance checks after construction.
        """
        shared_memory = wait_for_shared_memory(shm_name, retries)
        self._mmap = shared_memory._mmap
        self._packer = msgpack.Packer(default=serialize, use_bin_type=True)
        self._shared_memory = shared_memory
        self._stop_waiting = set([Request.kNone, Request.kError])
        self._unpacker = msgpack.Unpacker(raw=False)
        if perf_checks:
            self.__perf_checks()

    def __perf_checks(self):
        packer_cls = str(msgpack.Packer)
        unpacker_cls = str(msgpack.Unpacker)
        if "fallback" in packer_cls or "fallback" in unpacker_cls:
            # See https://github.com/upkie/upkie/issues/377
            raise PerformanceIssue("msgpack is running in pure Python")

    def __del__(self):
        """!
        Close memory mapping.

        Note that the spine process will unlink the shared memory object, so we
        don't unlink it here.
        """
        if hasattr(self, "_shared_memory"):  # handle ctor exceptions
            self._shared_memory.close()

    def get_first_observation(self) -> dict:
        r"""!
        Get first observation after a reset.

        \return Observation dictionary.
        """
        self.get_observation()  # pre-reset observation, skipped
        return self.get_observation()

    def set_action(self, action: dict) -> dict:
        r"""!
        Set action for the spine to process.

        \param[in] action Action dictionary.
        \return Observation dictionary.
        """
        self._wait_for_spine()
        self._write_dict(action)
        self._write_request(Request.kAction)
        self._wait_for_spine()
        observation = self._read_dict()
        return observation

    def start(self, config: dict) -> None:
        r"""!
        Reset the spine to a new configuration.

        \param[in] config Configuration dictionary.
        """
        self._wait_for_spine()
        self._write_dict(config)
        self._write_request(Request.kStart)

    def stop(self) -> None:
        """!
        Tell the spine to stop all actuators.
        """
        self._wait_for_spine()
        self._write_request(Request.kStop)

    def _read_request(self) -> int:
        """!
        Read current request from shared memory.
        """
        self._mmap.seek(0)
        return int.from_bytes(self._mmap.read(4), byteorder=sys.byteorder)

    def _read_dict(self) -> dict:
        r"""!
        Read dictionary from shared memory.

        \return Observation dictionary.
        """
        assert self._read_request() == Request.kNone
        self._mmap.seek(0)
        self._mmap.read(4)  # skip request field
        size = int.from_bytes(self._mmap.read(4), byteorder=sys.byteorder)
        data = self._mmap.read(size)
        self._unpacker.feed(data)
        unpacked = 0
        last_dict = {}
        for observation in self._unpacker:
            last_dict = observation
            unpacked += 1
        assert unpacked == 1
        return last_dict

    def _wait_for_spine(self, timeout_ns: int = 100000000) -> None:
        r"""!
        Wait for the spine to signal itself as available, which it does by
        setting the current request to none in shared memory.

        \param timeout_ns Don't wait for more than this duration in
            nanoseconds.
        \raise SpineError If the request read from spine has an error flag.
        """
        stop = perf_counter_ns() + timeout_ns
        while self._read_request() not in self._stop_waiting:  # sets are fast
            # Fun fact: `not in set` is 3-4x faster than `!=` on the raspi
            # perf_counter_ns clocks ~1 us on the raspi
            if perf_counter_ns() > stop:
                raise TimeoutError(
                    "Spine did not process request within "
                    f"{timeout_ns / 1e6:.1f} ms, is it stopped?"
                )
        if self._read_request() == Request.kError:
            self._write_request(Request.kNone)
            raise SpineError("Invalid request, is the spine started?")

    def _write_request(self, request: int) -> None:
        """!
        Set request in shared memory.
        """
        self._mmap.seek(0)
        self._mmap.write(request.to_bytes(4, byteorder=sys.byteorder))

    def _write_dict(self, dictionary: dict) -> None:
        r"""!
        Set the shared memory to a given dictionary.

        \param dictionary Dictionary to pack and write.
        """
        assert self._read_request() == Request.kNone
        data = self._packer.pack(dictionary)
        size = len(data)
        self._mmap.seek(0)
        self._mmap.read(4)  # skip request field
        self._mmap.write(size.to_bytes(4, byteorder=sys.byteorder))
        self._mmap.write(data)
