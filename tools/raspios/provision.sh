#!/bin/sh
#
# SPDX-License-Identifier: Apache-2.0

# Configure CPU isolation
python /root/configure_cpu_isolation.py

# Disable the first-boot configuration assistant
rmdir /home/rpi-first-boot-wizard
systemctl stop userconfig
systemctl disable userconfig
systemctl mask userconfig

# Install Debian packages
export DEBIAN_FRONTEND=noninteractive
apt-get update && apt-get install -y python3-pip tmux vim

# Install packages from PyPI
pip install moteus-pi3hat upkie

# Set binary permissions
chmod 755 /usr/local/bin/hard_rezero
chmod 755 /usr/local/bin/micromamba
chmod 755 /usr/local/bin/pi3hat_spine
chmod 755 /usr/local/bin/stop_servos
chmod 755 /usr/local/bin/upkie_tool

# Configure micromamba
/usr/local/bin/micromamba config append channels conda-forge
/usr/local/bin/micromamba config append channels nodefaults
/usr/local/bin/micromamba config set channel_priority strict
