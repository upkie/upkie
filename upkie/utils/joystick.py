#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
#
# Source: https://gist.github.com/rdb/8864666
# Released by rdb under the Unlicense (unlicense.org)
# Based on information from:
# https://www.kernel.org/doc/Documentation/input/joystick-api.txt

import array
import os
import struct
from fcntl import ioctl

from ..exceptions import UpkieRuntimeError


class Joystick:
    r"""!
    Joystick class used by mock spines to mimick the C++ Joystick sensor.
    """

    # These constants were borrowed from linux/input.h
    AXIS_NAMES = {
        0x00: "x",
        0x01: "y",
        0x02: "z",
        0x03: "rx",
        0x04: "ry",
        0x05: "rz",
        0x06: "throttle",
        0x07: "rudder",
        0x08: "wheel",
        0x09: "gas",
        0x0A: "brake",
        0x10: "hat0x",
        0x11: "hat0y",
        0x12: "hat1x",
        0x13: "hat1y",
        0x14: "hat2x",
        0x15: "hat2y",
        0x16: "hat3x",
        0x17: "hat3y",
        0x18: "pressure",
        0x19: "distance",
        0x1A: "tilt_x",
        0x1B: "tilt_y",
        0x1C: "tool_width",
        0x20: "volume",
        0x28: "misc",
    }

    BUTTON_NAMES = {
        0x120: "trigger",
        0x121: "thumb",
        0x122: "thumb2",
        0x123: "top",
        0x124: "top2",
        0x125: "pinkie",
        0x126: "base",
        0x127: "base2",
        0x128: "base3",
        0x129: "base4",
        0x12A: "base5",
        0x12B: "base6",
        0x12F: "dead",
        0x130: "a",
        0x131: "b",
        0x132: "c",
        0x133: "x",
        0x134: "y",
        0x135: "z",
        0x136: "tl",
        0x137: "tr",
        0x138: "tl2",
        0x139: "tr2",
        0x13A: "select",
        0x13B: "start",
        0x13C: "mode",
        0x13D: "thumbl",
        0x13E: "thumbr",
        0x220: "dpad_up",
        0x221: "dpad_down",
        0x222: "dpad_left",
        0x223: "dpad_right",
        # XBox 360 controller uses these codes.
        0x2C0: "dpad_left",
        0x2C1: "dpad_right",
        0x2C2: "dpad_up",
        0x2C3: "dpad_down",
    }

    def __init__(self, device_path: str, prefix: str = "joystick"):
        r"""!
        Initialize joystick input reader.

        \param[in] device_path Path to the joystick device file on Linux.
        \param[in] prefix Prefix used when writing outputs to the observation
            dictionary.
        """
        jsdev = open(device_path, "rb")
        os.set_blocking(jsdev.fileno(), False)
        buf = array.array("B", [0] * 64)
        ioctl(jsdev, 0x80006A13 + (0x10000 * len(buf)), buf)  # JSIOCGNAME(len)

        # Get number of axes and buttons.
        buf = array.array("B", [0])
        ioctl(jsdev, 0x80016A11, buf)  # JSIOCGAXES
        num_axes = buf[0]

        buf = array.array("B", [0])
        ioctl(jsdev, 0x80016A12, buf)  # JSIOCGBUTTONS
        num_buttons = buf[0]

        # Get the axis map.
        buf = array.array("B", [0] * 0x40)
        ioctl(jsdev, 0x80406A32, buf)  # JSIOCGAXMAP

        axis_map = []
        axis_states = {}
        for axis in buf[:num_axes]:
            axis_name = Joystick.AXIS_NAMES.get(axis, "unknown(0x%02x)" % axis)
            axis_map.append(axis_name)
            axis_states[axis_name] = 0.0

        # Get the button map.
        buf = array.array("H", [0] * 200)
        ioctl(jsdev, 0x80406A34, buf)  # JSIOCGBTNMAP
        button_map = []
        button_states = {}
        for btn in buf[:num_buttons]:
            btn_name = Joystick.BUTTON_NAMES.get(btn, "unknown(0x%03x)" % btn)
            button_map.append(btn_name)
            button_states[btn_name] = 0

        self.axis_map = axis_map
        self.axis_states = axis_states
        self.button_map = button_map
        self.button_states = button_states
        self.jsdev = jsdev
        self.prefix = prefix

    def write(self, observation: dict):
        evbuf = self.jsdev.read(8)
        if evbuf:
            time, value, type, number = struct.unpack("IhBB", evbuf)
            if type & 0x80:  # initial
                pass
            if type & 0x01:
                button = self.button_map[number]
                if button:
                    self.button_states[button] = value
            if type & 0x02:
                axis = self.axis_map[number]
                if axis:
                    fvalue = value / 32767.0
                    self.axis_states[axis] = fvalue

        if self.button_states["b"]:
            raise UpkieRuntimeError("Stop button pressed")

        observation[self.prefix] = {
            "left_axis": [self.axis_states["x"], self.axis_states["y"]],
            "right_axis": [self.axis_states["rx"], self.axis_states["ry"]],
            "left_trigger": self.axis_states["throttle"],
            "right_trigger": self.axis_states["brake"],
        }
