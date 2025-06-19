#!/bin/sh
#
# SPDX-License-Identifier: Apache-2.0

# Default user is "pi" with password "raspberry"
echo "pi:raspberry" | chpasswd

# Configure CPU isolation
python /root/configure_cpu_isolation.py

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
chmod 755 /usr/local/bin/micromamba
chmod 755 /usr/local/bin/pi3hat_spine
chmod 755 /usr/local/bin/stop_servos
chmod 755 /usr/local/bin/upkie_tool
chmod 755 /usr/local/bin/vcgenall

# Configure micromamba
/usr/local/bin/micromamba config append channels conda-forge
/usr/local/bin/micromamba config append channels nodefaults
/usr/local/bin/micromamba config set channel_priority strict

# Configure micromamba for pi user
cat >> /home/pi/.bashrc << 'EOF'

# >>> mamba initialize >>>
# !! Contents within this block are managed by 'mamba init' !!
export MAMBA_EXE='/usr/local/bin/micromamba';
export MAMBA_ROOT_PREFIX='/home/pi/micromamba';
__mamba_setup="$("$MAMBA_EXE" shell hook --shell bash --root-prefix "$MAMBA_ROOT_PREFIX" 2> /dev/null)"
if [ $? -eq 0 ]; then
    eval "$__mamba_setup"
else
    alias micromamba="$MAMBA_EXE"  # Fallback on help from mamba activate
fi
unset __mamba_setup
# <<< mamba initialize <<<

alias conda="micromamba"
alias pst="ps -ao pcpu,cpuid,priority,pid,comm,command -T --sort=priority"
EOF
