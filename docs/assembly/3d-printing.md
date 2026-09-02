# 2) 3D printing {#printing-3d}

[TOC]

The prerequisites for this stage are:

- 3D printer from the [bill of materials](\ref bill-of-materials)
- Power cables from [cables](\ref cables)
- Screw drivers from the [bill of materials](\ref bill-of-materials)

## 2.1) Slice 3D printing projects

3D printing project (3MF) files are provided for each part in the [`upkie_parts`](https://github.com/upkie/upkie_parts) repository. They include all configuration details (speed, infill, layer height, etc.) required for printing. All parts were printed multiple times with a [Prusa i3 MK3S+](https://www.prusa3d.com/product/original-prusa-i3-mk3s-kit-3/). If you have a different 3D printer, you can also import STL mesh files in the `upkie_parts` repository and apply the settings described below.

Parts are designed to be printed in PETG (which has better impact resistance than PLA) with a 0.2 mm layer height. Infill varies from 15% to 30%, but most of these values are guesstimates that don't come from observing parts breaking after robot falls. The only exception to this is the wheel hub, where there is an explicit infill modifier on the hex coupler connector.

| Part name                     | Infill          |
|-------------------------------|-----------------|
| Back cover (bottom)           | 20%             |
| Back cover (top)              | 20%             |
| Back plate (bottom)           | 60%             |
| Back plate (top)              | 60%             |
| Battery shore plug (optional) | 40%             |
| Battery stud (left)           | 30%             |
| Battery stud (right)          | 30%             |
| Bottom plate                  | 40%             |
| Face plate                    | 60%             |
| Head plate                    | 40%, cones 100% |
| Side plate                    | 60%             |
| Stiffener (bottom)            | 20%             |
| Stiffener (top)               | 20%             |

Slice each part with the proper settings.

## 2.2) Print 3D printed parts

Print all the parts sliced in the previous step. In total, you should print:

1. Torso parts from the [`torso/`](https://github.com/upkie/parts/tree/8e539b825affe10ad872741564bdc1c19f4df965/torso) directory:
    - Back covers: 1×
    - Back plates: 1×
    - Battery shore plug: 1×
    - Battery stud: 1×
    - Bottom plate: 1×
    - Face plate: 1×
    - Head plate: 1×
    - Side plate: 2×
    - Stiffener bottom: 1×
    - Stiffener top: 1×
2. Leg parts from the [`legs/`](https://github.com/upkie/parts/tree/8e539b825affe10ad872741564bdc1c19f4df965/legs) directory:
    - Ankle: 2×
    - Femur: 2×
    - Horn: 4×
    - Knee: 2×
    - Tibia: 2×
    - Wheel hub: 2×

## 2.3) Battery stud

<a href="battery-stud.jpg">
<img src="battery-stud.jpg" height="150" align="right">
</a>

There are more pictures of this step in the [Battery stud assembly](https://github.com/upkie/upkie/discussions/91) discussion thread.

Note that the two halves of the battery stud are not symmetric.

1. Identify the battery-stud half that mates with the **positive** pole of the battery
2. Identify the battery-stud half that mates with the **negative** pole of the battery
3. Make a red power cable go through the hole of the positive battery stud half, then solder a battery spring to it
4. Repeat the process with a black power cable go through the other half
5. Push the two battery springs into place (their flat parts should be flush with the 3D printed part, [like so](https://github.com/upkie/upkie/discussions/91#discussioncomment-6499911))
6. Solder the red and black cable into the XT90-S connector
7. Insert two M3x5x4 heat-set inserts in the corresponding holes of each battery stud half
8. Use two M3x8 screws to assemble the two battery stud halves together

## 2.4) Heat-set inserts

Check out [Threading 3D Printed Parts: How To Use Heat-Set Inserts](https://hackaday.com/2019/02/28/threading-3d-printed-parts-how-to-use-heat-set-inserts/) if it is your first time with heat-set inserts. Once you're ready, proceed to insert them into the following parts:

| Part               | Qty | Insert type (screw x OD x depth) | Number of inserts / part | Total num. of inserts |
|--------------------|-----|----------------------------------|--------------------------|-----------------------|
| Battery stud left  | 1   | M3x5x4   | 1  | 1  |
| Battery stud right | 1   | M3x5x4   | 1  | 1  |
| Back covers        | 2   | M2.5x4x4 | 5  | 10 |
| Bottom plate       | 1   | M2.5x4x4 | 4  | 1  |
| Femur              | 2   | M3x5x4   | 4  | 8  |
| Head plate         | 1   | M2.5x4x4 | 4  | 4  |
| Side plate         | 2   | M3x5x4   | 26 | 52 |
| Tibia              | 2   | M3x5x4   | 4  | 8  |

<sub>Ask questions about this step in [Hardware discussions](https://github.com/upkie/upkie/discussions/categories/hardware).</sub>

---

**Previous:** \ref getting-started — **Next:** \ref raspberry-pi-setup
