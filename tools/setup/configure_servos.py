#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria
# SPDX-License-Identifier: Apache-2.0
#
# This file incorporates work covered by the following copyright and
# permission notice:
#
#     setup-system.py from github.com:mjbots/quad
#     Copyright 2018-2020 Josh Pieper
#     License: Apache-2.0


"""Configure all servos connected to an Upkie wheeled biped."""

import datetime
import os
import subprocess
import sys
import time
from typing import Union

ORIG_SUFFIX = time.strftime(".orig-%Y%m%d-%H%M%S")

logging_depth = 0


def log_message(message: str, indent: int = 0) -> None:
    """Log a single message.

    Args:
        message: Message to log.
        indent: Indentation level.
    """
    logging_indent = " | " * (logging_depth + indent)
    now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
    print(f"{now}: {logging_indent}{message}")


def log_method(func):
    """Decorator to log function calls."""

    def wrapped_function(*args, **kwargs):
        global logging_depth
        try:
            logging_depth += 1
            args_repr = (
                f"'{args[0]}'" + (", ..." if len(args) > 1 else "")
                if args
                else ""
            )
            log_message(f"{func.__name__}({args_repr})", indent=-1)
            result = func(*args, **kwargs)
            return result
        finally:
            logging_depth -= 1

    return wrapped_function


@log_method
def run(*args, **kwargs):
    """Run a subprocess."""
    subprocess.check_call(*args, shell=True, **kwargs)


def configure_servo(id: int, param: str, value: Union[float, str]):
    """Configure a moteus controller.

    Args:
        id: Identifier of the servo.
        param: Parameter to configure.
        value: Value corresponding to the parameter.
    """
    moteus_command = f"conf set {param} {value}"
    shell_command = (
        f'echo "{moteus_command}" | '
        f'sudo moteus_tool --pi3hat-cfg "1=1,2,3;2=4,5,6" -t {id} -c'
    )
    run(shell_command)


def write_configuration(id: int):
    """Write configuration to a moteus controller.

    Args:
        id: Identifier of the servo that should write its configuration.
    """
    shell_command = (
        f'echo "conf write" | '
        f'sudo moteus_tool --pi3hat-cfg "1=1,2,3;2=4,5,6" -t {id} -c'
    )
    run(shell_command)


if __name__ == "__main__":
    if os.geteuid() != 0:
        args = ["sudo", "-E", sys.executable] + sys.argv + [os.environ]
        os.execlpe("sudo", *args)

    # TODO(scaron): upkie layout for Python as well
    left_hip = 1
    left_knee = 2
    left_wheel = 3
    right_hip = 4
    right_knee = 5
    right_wheel = 6
    all_servos = (1, 2, 3, 4, 5, 6)

    for hip in (left_hip, right_hip):
        configure_servo(hip, "servopos.position_max", 0.2)
        configure_servo(hip, "servopos.position_min", -0.2)
        configure_servo(hip, "servo.max_velocity", 2.0)

    for knee in (left_knee, right_knee):
        configure_servo(knee, "servopos.position_max", 0.4)
        configure_servo(knee, "servopos.position_min", -0.4)
        configure_servo(knee, "servo.max_velocity", 2.0)

    for wheel in (left_wheel, right_wheel):
        configure_servo(wheel, "servopos.position_max", "NaN")
        configure_servo(wheel, "servopos.position_min", "NaN")
        configure_servo(wheel, "servo.max_velocity", 8.0)

    # https://github.com/mjbots/moteus/blob/38d688a933ce1584ee09f2628b5849d5e758ac21/docs/reference.md#servodefault_velocity_limit--servodefault_accel_limit
    for servo in all_servos:
        configure_servo(servo, "servo.default_velocity_limit", "NaN")
        configure_servo(servo, "servo.default_accel_limit", "NaN")

    for servo in all_servos:
        write_configuration(servo)
