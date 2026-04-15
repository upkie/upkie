#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

"""Unit tests for the Joystick utility class."""

import struct
import unittest
from unittest.mock import MagicMock, patch

from upkie.exceptions import UpkieRuntimeError
from upkie.utils.joystick import Joystick


def make_mock_joystick(prefix: str = "joystick") -> Joystick:
    """Create a Joystick instance with all device I/O mocked out.

    ioctl calls are no-ops, so num_axes and num_buttons both read as 0 from
    their respective buffers. The default axis_states and button_states dicts
    are still populated inside __init__, so write() works normally.
    """
    mock_file = MagicMock()
    mock_file.fileno.return_value = 3
    mock_file.read.return_value = b""  # no pending events by default
    with (
        patch("builtins.open", return_value=mock_file),
        patch("os.set_blocking"),
        patch("upkie.utils.joystick.ioctl"),
    ):
        joystick = Joystick("/dev/input/js0", prefix=prefix)
    return joystick


class JoystickTestCase(unittest.TestCase):
    def setUp(self):
        self.joystick = make_mock_joystick()

    def test_write_returns_expected_keys(self):
        """write() adds a dict with the expected observation keys."""
        observation = {}
        self.joystick.write(observation)
        self.assertIn("joystick", observation)
        expected_keys = {
            "cross_button",
            "left_axis",
            "left_button",
            "left_trigger",
            "pad_axis",
            "right_axis",
            "right_button",
            "right_trigger",
            "square_button",
            "triangle_button",
        }
        self.assertEqual(set(observation["joystick"].keys()), expected_keys)

    def test_write_default_values(self):
        """write() fills buttons with 0 and axes with [0, 0] by default."""
        observation = {}
        self.joystick.write(observation)
        joy = observation["joystick"]
        self.assertEqual(joy["cross_button"], 0)
        self.assertEqual(joy["left_button"], 0)
        self.assertEqual(joy["right_button"], 0)
        self.assertEqual(joy["square_button"], 0)
        self.assertEqual(joy["triangle_button"], 0)
        self.assertEqual(joy["left_axis"], [0.0, 0.0])
        self.assertEqual(joy["right_axis"], [0.0, 0.0])
        self.assertEqual(joy["pad_axis"], [0.0, 0.0])

    def test_write_custom_prefix(self):
        """write() uses the prefix given at construction."""
        joystick = make_mock_joystick(prefix="gamepad")
        observation = {}
        joystick.write(observation)
        self.assertIn("gamepad", observation)
        self.assertNotIn("joystick", observation)

    def test_b_button_state_raises(self):
        """write() raises UpkieRuntimeError when the B button state is set."""
        self.joystick.button_states["b"] = 1
        with self.assertRaises(UpkieRuntimeError):
            self.joystick.write({})

    def test_b_button_event_raises(self):
        """write() raises UpkieRuntimeError when it reads a B-button event.

        Button number 1 is the B / circle / red stop button. The event buffer
        format is: uint32 time, int16 value, uint8 type, uint8 number.
        type=0x01 means JS_EVENT_BUTTON.
        """
        self.joystick.button_map = ["a", "b"]  # button index 1 → "b"
        b_pressed = struct.pack("IhBB", 0, 1, 0x01, 1)
        self.joystick.jsdev.read.return_value = b_pressed
        with self.assertRaises(UpkieRuntimeError):
            self.joystick.write({})

    def test_other_buttons_do_not_raise(self):
        """write() does not raise for buttons other than B."""
        for btn in ("a", "x", "y", "tl", "tr"):
            self.joystick.button_states[btn] = 1
        observation = {}
        self.joystick.write(observation)  # must not raise
        self.assertIn("joystick", observation)

    def test_pad_axis_event_appears_in_observation(self):
        """write() reflects hat0x/hat0y axis events in pad_axis.

        Axis numbers 0x10 (hat0x) and 0x11 (hat0y) map to pad_axis[0] and
        pad_axis[1]. type=0x02 is JS_EVENT_AXIS. value=16383 → ~0.5.
        """
        self.joystick.axis_map = [
            "x",
            "y",
            "z",
            "rx",
            "ry",
            "rz",
            "throttle",
            "brake",
            "hat0x",
            "hat0y",
        ]
        # hat0x = 32767 → 1.0
        hat0x_event = struct.pack("IhBB", 0, 32767, 0x02, 8)
        self.joystick.jsdev.read.return_value = hat0x_event
        observation = {}
        self.joystick.write(observation)
        self.assertAlmostEqual(observation["joystick"]["pad_axis"][0], 1.0)

        # hat0y = -32767 → -1.0
        hat0y_event = struct.pack("IhBB", 0, -32767, 0x02, 9)
        self.joystick.jsdev.read.return_value = hat0y_event
        observation = {}
        self.joystick.write(observation)
        self.assertAlmostEqual(observation["joystick"]["pad_axis"][1], -1.0)


if __name__ == "__main__":
    unittest.main()
