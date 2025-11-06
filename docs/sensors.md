# Sensors {#sensors}

[TOC]

Spines pull sensor measurements directly from host devices and files. This page lists the outputs of available sensors, using shorthands `a.b.c` for nested dictionary keys `observation["a"]["b"]["c"]`.

## CPU Temperature

- Sensor: [CpuTemperature](\ref upkie::cpp::sensors::CpuTemperature)
- Spine observation key: `cpu_temperature`

| Observation key | Description |
|-----------------|-------------|
| `cpu_temperature` | CPU temperature in degrees Celsius |

## Joystick

- Sensor: [Joystick](\ref upkie::cpp::sensors::Joystick)
- Spine observation key: `joystick`

| Observation key | Description |
|-----------------|-------------|
| `joystick.cross_button` | Bottom button (cross on PS4, A on Xbox) |
| `joystick.left_axis` | Left analog joystick position (x, y) |
| `joystick.left_button` | Left button (L1 on PS4, L on Xbox) |
| `joystick.left_trigger` | Left trigger position |
| `joystick.pad_axis` | Directional pad position (x, y) |
| `joystick.right_axis` | Right analog joystick position (x, y) |
| `joystick.right_button` | Right button (R1 on PS4, R on Xbox) |
| `joystick.right_trigger` | Right trigger position |
| `joystick.square_button` | Left button (square on PS4, Y on Xbox) |
| `joystick.triangle_button` | Top button (triangle on PS4, X on Xbox) |

## Keyboard

- Sensor: [Keyboard](\ref upkie::cpp::sensors::Keyboard)
- Spine observation key: `keyboard`

| Observation key | Description |
|-----------------|-------------|
| `keyboard.key_pressed` | Whether a keyboard key is currently pressed |
| `keyboard.up` | Whether the Up key is pressed |
| `keyboard.down` | Whether the Down key is pressed |
| `keyboard.left` | Whether the Left key is pressed |
| `keyboard.right` | Whether the Right key is pressed |
| `keyboard.a` | Whether the A key is pressed |
| `keyboard.d` | Whether the D key is pressed |
| `keyboard.s` | Whether the S key is pressed |
| `keyboard.w` | Whether the W key is pressed |
| `keyboard.x` | Whether the X key is pressed |
| `keyboard.unknown` | Whether an unknown key is pressed |
