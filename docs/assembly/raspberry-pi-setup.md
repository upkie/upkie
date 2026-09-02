# 3) Raspberry Pi setup {#raspberry-pi-setup}

[TOC]

The prerequisites for this stage are:

- Internet connection on a Wireless LAN
- microSD card from the [bill of materials](\ref bill-of-materials)
- Raspberry Pi from the [bill of materials](\ref bill-of-materials)

## 3.1) Write the OS to the microSD card

Write the file `raspios_upkie.img` to the microSD card.

This image is built by the automated image builder in the `tools/raspios/` directory of the repository. Check out its scripts if you are curious about what the robot's operating system ships with, or if you want to customize it.

## 3.2) Connect to the local network

- Connect screen, keyboard and mouse to the Raspberry Pi and boot it. The default credentials are:
    - User: "pi"
    - Password: "raspberry"
- Connect it to your local network.
- In what follows, we will assume the robot's IP address is 192.168.0.42.

## 3.3) Convenience SSH configuration

You should now be able to log into to the robot by connecting to its Wi-Fi access point and running:

```console
$ ssh pi@192.168.0.42
```

For convenience, you can add the following to your `~/.ssh/config` file, replacing "upkie" with your preferred nickname:

```
Host upkie
Hostname 192.168.0.42
User pi
```

Makefile rules such as `make upload` assume such a `upkie` host is configured.

<sub>Ask questions about this step in [Software discussions](https://github.com/upkie/upkie/discussions/categories/software).</sub>

---

**Previous:** \ref printing-3d — **Next:** \ref electronics-testing
