#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2022 Stéphane Caron
# Copyright 2023 Inria
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# This file incorporates work covered by the following copyright and
# permission notice:
#
#     setup-system.py from github.com:mjbots/quad
#     Copyright 2018-2020 Josh Pieper
#     License: Apache-2.0


"""Set up a Raspberry Pi 4b to operate a Wi-Fi access point.

The base operating system should already be installed. This script is intended
to be run as root, like: ``sudo ./setup-wifi-ap.py``.
"""

import argparse
import datetime
import os
import pathlib
import shutil
import subprocess
import time

ORIG_SUFFIX = time.strftime(".orig-%Y%m%d-%H%M%S")

# Log each setup step, in case
# ============================

logging_depth = 0


def log_message(message: str, indent: int = 0) -> None:
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
                f'"{args[0]}"' + (", ..." if len(args) > 1 else "")
                if args
                else ""
            )
            log_message(f"{func.__name__}({args_repr})", indent=-1)
            result = func(*args, **kwargs)
            return result
        finally:
            logging_depth -= 1

    return wrapped_function


# Command-line arguments
# ======================


def parse_command_line_arguments() -> argparse.Namespace:
    """Parse command-line arguments.

    Returns:
        Namespace resulting from parsing command-line arguments.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "ssid",
        help="SSID of the robot's Wi-Fi network",
    )
    parser.add_argument(
        "wpa_passphrase",
        help="Password of the robot's Wi-Fi network",
    )
    parser.add_argument(
        "country_code",
        help="Two-letter country code for Wi-Fi compliance (e.g. FR or UK)",
    )
    parser.add_argument(
        "wlan_prefix",
        help="IPv4 address prefix (first three bytes) "
        "for the robot's Wi-Fi network interface (for instance 192.168.0)",
    )
    parser.add_argument(
        "eth_prefix",
        help="IPv4 address prefix (first three bytes) "
        "for the robot's wired network interface (for instance 192.168.1)",
    )
    return parser.parse_args()


# Utility functions
# =================


@log_method
def run(*args, **kwargs):
    subprocess.check_call(*args, shell=True, **kwargs)


@log_method
def ensure_present(filename, line):
    """Ensure a given line is present in a named file, and add it if not.

    Args:
        filename: Path to file.
        line: Line that should be present in the file's content.
    """
    current_content = [
        x.strip() for x in open(filename, encoding="utf-8").readlines()
    ]
    if line.strip() in current_content:
        return
    shutil.copy(filename, filename + ORIG_SUFFIX)
    log_message(f"Adding: {line}")
    with open(filename, "a", encoding="utf-8") as f:
        f.write(line + "\n")


@log_method
def ensure_contents(filename, contents):
    """Ensure the given file has exactly the given contents.

    Args:
        filename: Path to file.
        contents: File contents.
    """
    pathlib.Path(filename).parent.mkdir(parents=True, exist_ok=True)

    if os.path.exists(filename):
        existing = open(filename, encoding="utf-8").read()
        if existing == contents:
            return
        shutil.copy(filename, filename + ORIG_SUFFIX)

    log_message("Updating file content")

    with open(filename, "w", encoding="utf-8") as f:
        f.write(contents)


@log_method
def set_config_var(name, value):
    """Set the given variable in /boot/config.txt.

    Args:
        name: Variable name.
        value: Value to set.
    """
    contents = open("/boot/config.txt", encoding="utf-8").readlines()

    new_value = "{}={}".format(name, value)

    maybe_value = [x for x in contents if x.startswith("{}=".format(name))]
    if len(maybe_value) == 1 and maybe_value[0].strip() == new_value:
        return

    new_contents = [
        x for x in contents if not x.startswith("{}=".format(name))
    ] + [new_value + "\n"]

    shutil.copy("/boot/config.txt", "/boot/config.txt" + ORIG_SUFFIX)

    log_message("Updating {name} to {value} in /boot/config.txt")

    open("/boot/config.txt", "w", encoding="utf-8").write(
        "".join([x for x in new_contents])
    )


# Setup steps
# ===========


@log_method
def install_packages():
    """Install packages for the access point."""
    run("apt-get install --yes hostapd dnsmasq")


@log_method
def configure_interfaces(wlan_prefix, eth_prefix):
    ensure_contents(
        "/etc/network/interfaces",
        """# interfaces(5) file used by ifup(8) and ifdown(8)

# Please note that this file is written to be used with dhcpcd
# For static IP, consult /etc/dhcpcd.conf and 'man dhcpcd.conf'

# Include files from /etc/network/interfaces.d:
source-directory /etc/network/interfaces.d
""",
    )
    ensure_contents(
        "/etc/dhcpcd.conf",
        f"""# Configuration for dhcpcd.
# See dhcpcd.conf(5) for details.

# Allow users of this group to interact with dhcpcd via the control socket.
#controlgroup wheel

# Inform the DHCP server of our hostname for DDNS.
hostname

# Use the hardware address of the interface for the Client ID.
clientid
# or
# Use the same DUID + IAID as set in DHCPv6 for DHCPv4 ClientID as per RFC4361.
# Some non-RFC compliant DHCP servers do not reply with this set.
# In this case, comment out duid and enable clientid above.
#duid

# Persist interface configuration when dhcpcd exits.
persistent

# Rapid commit support.
# Safe to enable by default because it requires the equivalent option set
# on the server to actually work.
option rapid_commit

# A list of options to request from the DHCP server.
option domain_name_servers, domain_name, domain_search, host_name
option classless_static_routes
# Respect the network MTU. This is applied to DHCP routes.
option interface_mtu

# Most distributions have NTP support.
#option ntp_servers

# A ServerID is required by RFC2131.
require dhcp_server_identifier

# Generate SLAAC address using the Hardware Address of the interface
#slaac hwaddr
# OR generate Stable Private IPv6 Addresses based from the DUID
slaac private

# Static IP configuration for debugging
interface eth0
static ip_address={eth_prefix}.42/24
static routers={eth_prefix}.1

# Wireless access point configuration for the robot
interface wlan0
nohook wpa_supplicant
static ip_address={wlan_prefix}.42/24
static routers={wlan_prefix}.1
""",
    )


@log_method
def configure_hostapd(ssid, wpa_passphrase, country_code):
    ensure_contents(
        "/etc/hostapd/hostapd.conf",
        f"""country_code={country_code}

interface=wlan0
driver=nl80211
ssid={ssid}
hw_mode=a
channel=36
ieee80211n=1
require_ht=1
ieee80211ac=1
require_vht=1
ieee80211d=1
ieee80211h=0

ht_capab=[HT40+][SHORT-GI-20][DSSS_CK-40][MAX-AMSDU-3839]
vht_capab=[MAX-MDPU-3895][SHORT-GI-80][SU-BEAMFORMEE]

vht_oper_chwidth=1
vht_oper_centr_freq_seg0_idx=42

wmm_enabled=0
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0

wpa=2
wpa_passphrase={wpa_passphrase}
wpa_key_mgmt=WPA-PSK
wpa_pairwise=TKIP
rsn_pairwise=CCMP
""",
    )
    ensure_present(
        "/etc/default/hostapd", 'DAEMON_CONF="/etc/hostapd/hostapd.conf"'
    )
    run("systemctl unmask hostapd")
    run("systemctl enable hostapd")
    run("systemctl start hostapd")
    run("systemctl daemon-reload")


@log_method
def configure_dnsmasq(wlan_prefix):
    ensure_contents(
        "/etc/dnsmasq.conf",
        f"""interface=wlan0
listen-address=::1,127.0.0.1,{wlan_prefix}.42
domain-needed
bogus-priv
dhcp-range={wlan_prefix}.100,{wlan_prefix}.110,255.255.255.0,24h
""",
    )


@log_method
def configure_access_point(
    ssid,
    wpa_passphrase,
    country_code,
    wlan_prefix,
    eth_prefix,
):
    """Configure the raspi as an access point.

    Args:
        ssid: SSID of the Wi-Fi network.
        wpa_passphrase: Wi-Fi password.
        country_code: Two-letter country code for compliance.
        wlan_prefix: IPv4 address prefix (three first bytes) for the robot's
            Wi-Fi network, e.g. "192.168.0".
        eth_prefix: IPv4 address prefix (three first bytes) for the robot's
            wired network, e.g. "192.168.1".

    Note:
        The robot's IP suffix on both eth0 and wlan0 interfaces is 42. IP
        address suffixes on the wireless network are assigned from 100 to 110.
    """
    run("rfkill unblock all")
    configure_interfaces(wlan_prefix, eth_prefix)
    configure_hostapd(ssid, wpa_passphrase, country_code)
    configure_dnsmasq(wlan_prefix)


if __name__ == "__main__":
    if os.getuid() != 0:
        raise RuntimeError("must be run as root")
    args = parse_command_line_arguments()
    install_packages()
    configure_access_point(
        args.ssid,
        args.wpa_passphrase,
        args.country_code,
        args.wlan_prefix,
        args.eth_prefix,
    )
