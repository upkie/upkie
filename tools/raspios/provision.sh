#!/bin/sh
#
# SPDX-License-Identifier: Apache-2.0

# Default user is "pi" with password "raspberry"
echo "pi:raspberry" | chpasswd

# Configure CPU isolation
python /root/configure_cpu_isolation

# Disable the first-boot configuration assistant
rmdir /home/rpi-first-boot-wizard
systemctl stop userconfig
systemctl disable userconfig
systemctl mask userconfig
rm /etc/xdg/autostart/piwiz.desktop

# Prepare pi user and its home directrory
cp /root/WELCOME /home/pi/WELCOME && chown pi:pi /home/pi/WELCOME
rm -rf /home/pi/Bookshelf

# Install Debian packages
export DEBIAN_FRONTEND=noninteractive
apt-get update && apt-get install -y python3-numpy python3-pip tmux vim

# Install packages from PyPI
pip install moteus-pi3hat upkie

# Set binary permissions
chmod 755 /usr/local/bin/hard_rezero
chmod 755 /usr/local/bin/pi3hat_spine
chmod 755 /usr/local/bin/stop_servos
chmod 755 /usr/local/bin/upkie_tool
chmod 755 /usr/local/bin/vcgenall
