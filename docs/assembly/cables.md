# Cables {#cables}

[TOC]

## Comms cables

<a href="comms-cables.jpg">
<img src="comms-cables.jpg" width="150" align="right">
</a>

Communications from and to servos are transmitted as a differential signal over twisted pairs of wires. These cables have the following specs:

- **Connectors:** female JST PH-3 (3-pin JST connectors with 2.0 mm (PH) pin-to-pin pitch)
- **Wire gauge:** 26 AWG (24 AWG to 32 AWG should work)
- **Wire length:** 42 cm
- Tip: order PVC rather than Silicon wires (they are easier to twist)

Comms cable have female-female connectors, that is, female connectors on both sides. Pins inside are ordered as follows, when looking at the back of the connector (from the cable to the connector):

- Left: `CAN_H` (typically blue)
- Center: `CAN_L` (typically green)
- Right: `GND` (typically black) ⚠️ we leave this one **empty**

See also the [JST PH-3 CAN](https://github.com/mjbots/power_dist/blob/main/docs/reference.md#jst-ph-3-can) section in the power-dist board reference.

## Power cables

<a href="power-cables.jpg">
<img src="power-cables.jpg" width="150" align="right">
</a>

The power bus is obtained by daisy-chaining bigger cables through servos and electronics to the battery. These cables don't need to be twisted.

- **Connectors:** XT-30
- **Wire gauge:** 14 AWG (16 AWG works too)

We have two types of power cables, as depicted in the pictures to the right:

- Power cables between actuators:
    - Connectors: female-female
    - Wire length: 42 cm
- Power cables to the power dist board:
    - Connectors: female-male
    - Wire length: 20 cm

<sub>Ask questions about these cables in [Hardware discussions](https://github.com/upkie/upkie/discussions/categories/hardware).</sub>
