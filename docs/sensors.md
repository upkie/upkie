# Sensors {#sensors}

[TOC]

Spines pull sensor measurements directly from host devices and files. This page lists the outputs of available sensors, using shorthands `a.b.c` for nested dictionary keys `observation["a"]["b"]["c"]`.

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
