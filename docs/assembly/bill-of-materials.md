# Bill of materials {#bill-of-materials}

[TOC]

Online retail links, written between brackets like [[1](#optional-components), [2](#optional-components)], are only provided as examples and for reference. These links may break in the future.

| Short name         | Part name and requirements                                 | Quantity  | Suppliers     |
|--------------------|------------------------------------------------------------|-----------|---------------|
| 3D printer         | FDM 3D printer such as a [Prusa i3 MK3S+](https://www.prusa3d.com/product/original-prusa-i3-mk3s-3d-printer-3/) | 1 | your preference |
| Battery            | RYOBI 18V compatible battery                               | 1         | online retail [[1](https://www.amazon.fr/batterie-OnePlus-chargeur-rapide-RC18120-150/dp/B075LFR84B/)] |
| Battery spring     | Battery contact spring                                     | 2         | [Digi-Key](https://www.digikey.fr/en/products/detail/5205/36-5205-ND/2745874?itemSeq=376681212) |
| DC fan             | DC FAN 24V 1.8W 60x60x15mm                                 | 1         | online retail [[1](https://www.amazon.fr/Ventilateur-60x60x15mm-4dBA-Sunon-MF60152V11000UA99/dp/B07ZBS6W6Z/)] |
| Heat shrink        | Heat shrink tubing, 3:1 ratio, standard sizes              | 2 m       | online retail |
| Heat shrink large  | Heat shrink tubing, 2:1 ratio, 19 mm diameter              | 1 m       | online retail |
| Hex spacer         | M2x12 + 3 male hex spacer                                  | 4         | comes with pi3hat |
| Hex wrench         | 17 mm RC Hex Wrench                                        | 1         | [3D print](https://www.printables.com/model/59120-17mm-rc-hex-wrench/comments/151854) |
| JST connector      | JST PH-3 2.0 mm 3 pin connector, see [cables](\ref cables)  | 14        | come with mjbots products |
| Leg power cable    | Female-female power cable, length 400 mm                   | 4         | see [cables](\ref cables) |
| M2.5 inserts       | M2.5x4x4 heat set inserts, M2.5, OD 4 mm, length 4 mm      | 8         | online retail |
| M2.5 screws        | M2x8 socket head screws, M2.5, length 8 mm                 | 10        | online retail |
| M2.5x4 screws      | M2.5x4 screws, M2.5, length 4 mm                           | 4         | comes with pi3hat |
| M2.5x6 screws      | M2.5 screws, M2.5, length 6 mm                             | 4         | online retail |
| M2.5x8 screws      | M2.5x8 screws, M2.5, length 8 mm                           | 10        | online retail |
| M3 inserts         | M3x5x4 heat set inserts, M3, OD 5mm, length 4 mm           | 100       | online retail |
| M3x10 screws       | M3x10 socket head screws, M3, length 10 mm                 | 12        | online retail |
| M3x12 screws       | M3x12 socket head screws, M3, length 12 mm                 | 22        | online retail |
| M3x8 screws        | M3x8 socket head screws, M3, length 8 mm                   | 50        | online retail |
| Micro HDMI cable   | Any micro HDMI cable that works with your monitor          | 1         | online retail [[1](https://www.raspberrypi.com/products/micro-hdmi-to-standard-hdmi-a-cable/)] |
| MicroSD card       | microSDXC card, U3, V30, A2                                | 1         | online retail |
| Pi3hat             | mjbots pi3hat, r4.4b+                                      | 1         | [mjbots](https://mjbots.com/products/mjbots-pi3hat-r4-4b) |
| Power dist         | mjbots power dist, r4.3b+                                  | 1         | [mjbots](https://mjbots.com/products/mjbots-power-dist-r4-3b) |
| Qdd100             | mjbots qdd100 servo, beta 2+                               | 4         | [mjbots](https://mjbots.com/products/qdd100-beta-3) |
| Raspi              | Raspberry Pi 4 Model B                                     | 1         | see [raspberrypi.com](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/#find-reseller) |
| Raspi heat sink    | Aluminum Heatsink, for rpi 4b+                             | 1         | online retail |
| Terminator         | Resistor, 120 Ω, see [this note](https://github.com/mjbots/power_dist/blob/main/docs/reference.md#jst-ph-3-can) | 2 | online retail |
| Torso power cable  | Male-female power cable, length: 30 cm                     | 3         | see [cables](\ref cables) |
| USB gamepad        | USB gaming controller (or Bluetooth: [instructions](https://hackaday.io/project/185729/log/213395-locomotion-with-a-ps4-controller))   | 1         | online retail [[1](https://www.amazon.com/Logitech-Wireless-Nano-Receiver-Controller-Vibration/dp/B0041RR0TW/)] |
| Wheel              | RC 1:8 rubber tyre, see [RC wheels](#rc-wheels)            | 2         | online retail |
| Wheel actuator     | moteus developer kit, r4.11+                               | 2         | [mjbots](https://mjbots.com/products/moteus-r4-11-developer-kit) |
| Wheel hex coupler  | 17 mm Wheel Hex Coupler                                    | 2         | online retail [[1](https://www.amazon.com/Coupler-Wheel-Aluminum-Driver-Vehicle/dp/B07ZQJQM9V/)] |
| XT-30 connector    | XT30 connector, see [cables](\ref cables)                  | 20        | come with mjbots products |
| XT-90S connector   | XT90-S connector                                           | 1         | online retail [[1](https://www.amazon.com/FLY-RC-Connector-Anti-Spark-Adapter/dp/B0893CGLP7/)] |

## Optional components {#optional-components}

| Short name         | Part name and requirements   | Quantity  | Suppliers     |
|--------------------|------------------------------|-----------|---------------|
| RTC module         | DS3231 Chip                  | 1         | online retail |

## RC wheels {#rc-wheels}

* Outer diameter: 112 mm (software default, easy to adjust)
* Inner diameter: 80 mm
* Wheel drive hex: 17 mm
* Axle hole diameter: 12 mm

Changing the outer diameter is super easy (barely an inconvenience), you will only need to update your agent's configuration. Most agents are robust to cm-wide variations of this diameter.

Changing the inner diameter, wheel drive hex or axle hole diameter will require a corresponding update to the 3D printed [wheel hub](https://github.com/upkie/parts/tree/8e539b825affe10ad872741564bdc1c19f4df965/legs/wheel_hub). The wheel hub should be a tight fit into the wheel, which usually takes a few iterations to get right. If you select a different wheel drive hex or axle hole diameter, make sure to buy a matching hex coupler.

<sub>Ask questions about this page in [Hardware discussions](https://github.com/upkie/upkie/discussions/categories/hardware).</sub>
