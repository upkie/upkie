#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

"""!
Test the spine interface.
"""

import sys
import unittest
from multiprocessing.shared_memory import SharedMemory

import msgpack

from upkie.envs.backends.spine import Request, SpineInterface, serialize

wait_pre_monkeypatch = SpineInterface._wait_for_spine


class SpineInterfaceTestCase(unittest.TestCase):
    last_action: dict
    last_config: dict
    shm_name: str
    next_observation: dict
    spine: SpineInterface

    def setUp(self, shm_name="upkie", size=1024):
        """
        Initialize a new fixture instance.

        Args:
            shm_name: Name of shared memory file.
            size: Size of shared memory file in bytes.
        """
        shared_memory = SharedMemory(shm_name, create=True, size=size)
        self._mmap = shared_memory._mmap
        self._packer = msgpack.Packer(default=serialize, use_bin_type=True)
        self._shared_memory = shared_memory
        self._unpacker = msgpack.Unpacker(raw=False)
        self.last_action = {}
        self.last_config = {}
        self.shm_name = shm_name
        self.next_observation = {
            "servo": {
                "foo": {"position": 0.0},
                "bar": {"position": 1.0},
            }
        }

        def wait_monkeypatch(spine):
            """
            Monkey patch to the interface's `_wait_for_spine` function so that
            TestSpineInterface carries out the tasks expected by the spine.

            Args:
                spine: The spine interface waiting for a free request slot.
            """
            if self.__read_request() == Request.kAction:
                self.last_action = self.__read_dict()
                self.__write_observation(self.next_observation)
            elif self.__read_request() == Request.kStart:
                self.last_config = self.__read_dict()
            self.assertEqual(self.__read_request(), Request.kNone)

        self.assertTrue(hasattr(SpineInterface, "_wait_for_spine"))
        SpineInterface._wait_for_spine = wait_monkeypatch
        self.spine = SpineInterface(shm_name=shm_name, perf_checks=False)
        self.__write_request(Request.kNone)  # same as Spine constructor

    def tearDown(self) -> None:
        self._shared_memory.close()
        self._shared_memory.unlink()

    def __read_request(self) -> int:
        """
        Get shared memory request.
        """
        self._mmap.seek(0)
        return int.from_bytes(self._mmap.read(4), byteorder=sys.byteorder)

    def __write_request(self, request: int) -> None:
        """
        Set shared memory request.
        """
        self._mmap.seek(0)
        self._mmap.write(request.to_bytes(4, byteorder=sys.byteorder))

    def __write_observation(self, observation: dict) -> None:
        """
        Set shared memory dictionary to a given observation.

        Args:
            observation: Observation to write to shared memory.
        """
        data = self._packer.pack(observation)
        size = len(data)
        self._mmap.seek(0)
        self._mmap.read(4)
        self._mmap.write(size.to_bytes(4, byteorder=sys.byteorder))
        self._mmap.write(data)
        self.__write_request(Request.kNone)

    def __read_dict(self) -> dict:
        """
        Read dictionary from the memory shared with the spine interface.

        Returns:
            Dictionary read from shared memory.
        """
        self._mmap.seek(0)
        self._mmap.read(4)  # skip request
        size = int.from_bytes(self._mmap.read(4), byteorder=sys.byteorder)
        data = self._mmap.read(size)
        self._unpacker.feed(data)
        nb_unpacked = 0
        output = {}
        for unpacked in self._unpacker:
            output = unpacked
            nb_unpacked += 1
        self.assertEqual(nb_unpacked, 1)
        self.__write_request(Request.kNone)
        return output

    def test_reset(self):
        """
        Spine gets the proper configuration dictionary.
        """
        config = {"foo": {"bar": {"land": 42.0}}}
        self.assertEqual(self.__read_request(), Request.kNone)
        observation = self.spine.start(config)
        self.assertIsNotNone(observation)
        self.assertIsInstance(observation, dict)
        self.assertEqual(self.__read_request(), Request.kNone)
        self.assertEqual(self.last_config, config)

    def test_set_action(self):
        """
        Step sends an action we deserialize successfully, and returns a new
        observation.
        """
        action = {"servo": {"foo": {"position": 3.0}}}
        self.next_observation["servo"]["foo"]["position"] = 2.0
        self.assertEqual(self.__read_request(), Request.kNone)
        observation = self.spine.set_action(action)
        self.assertEqual(self.__read_request(), Request.kNone)
        self.assertEqual(self.last_action, action)
        self.assertEqual(observation, self.next_observation)

    def test_wait_times_out(self):
        """
        Disable monkeypatch to check that waiting for the spine (when there is
        no spine) times out.
        """
        SpineInterface._wait_for_spine = wait_pre_monkeypatch
        spine = SpineInterface(shm_name=self.shm_name, perf_checks=False)
        with self.assertRaises(TimeoutError):
            spine.start({"config": "empty"})
        with self.assertRaises(TimeoutError):
            spine.set_action({"foo": "bar"})
        with self.assertRaises(TimeoutError):
            spine.stop()


if __name__ == "__main__":
    unittest.main()
