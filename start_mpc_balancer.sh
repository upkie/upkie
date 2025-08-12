#!/bin/bash
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

SCRIPT=$(realpath "$0")
SCRIPTDIR=$(dirname "${SCRIPT}")

SYSTEM=$(uname -s)
ARCH=$(uname -m)

# Check if pixi is already installed system-wide
if command -v pixi >/dev/null 2>&1; then
    echo "✅ Pixi found in system PATH"
    PIXI_CMD="pixi"
elif [ -f cache/pixi ]; then
    echo "✅ Pixi found in cache"
    PIXI_CMD="./cache/pixi"
else
    echo "📥 Pixi not found, downloading to cache..."
    
    # Determine download URL based on system and architecture
    if [[ "$SYSTEM" == "Darwin" ]]; then
        if [[ "$ARCH" == "arm64" ]] || [[ "$ARCH" == arm* ]]; then
            PIXI_URL="https://github.com/prefix-dev/pixi/releases/latest/download/pixi-aarch64-apple-darwin"
        else
            PIXI_URL="https://github.com/prefix-dev/pixi/releases/latest/download/pixi-x86_64-apple-darwin"
        fi
    elif [[ "$SYSTEM" == "Linux" ]]; then
        if [[ "$ARCH" == "x86_64" ]]; then
            PIXI_URL="https://github.com/prefix-dev/pixi/releases/latest/download/pixi-x86_64-unknown-linux-musl"
        elif [[ "$ARCH" == "aarch64" ]] || [[ "$ARCH" == arm* ]]; then
            PIXI_URL="https://github.com/prefix-dev/pixi/releases/latest/download/pixi-aarch64-unknown-linux-musl"
        else
            echo "❌ Unsupported Linux architecture: $ARCH"
            exit 1
        fi
    else
        echo "❌ Unsupported operating system: $SYSTEM"
        exit 1
    fi
    
    # Create cache directory and download pixi
    mkdir -p cache
    echo "📥 Downloading pixi from $PIXI_URL..."
    
    if curl -L -o cache/pixi "$PIXI_URL"; then
        chmod +x cache/pixi
        echo "✅ Pixi downloaded to cache successfully"
        PIXI_CMD="./cache/pixi"
    else
        echo "❌ Failed to download pixi"
        exit 1
    fi
fi

# Function to cleanup background processes
cleanup() {
    echo "🛑 Stopping processes..."
    if [ -n "$SIMULATION_PID" ] && kill -0 "$SIMULATION_PID" 2>/dev/null; then
        kill "$SIMULATION_PID"
        wait "$SIMULATION_PID" 2>/dev/null
    fi
    if [ -n "$AGENT_PID" ] && kill -0 "$AGENT_PID" 2>/dev/null; then
        kill "$AGENT_PID"
        wait "$AGENT_PID" 2>/dev/null
    fi
    exit 0
}

# Set up signal handlers
trap cleanup INT TERM

cd "$SCRIPTDIR" || exit 1

# Start simulation as a background process
echo "🎮 Starting simulation..."
./start_simulation.sh &
SIMULATION_PID=$!

# Give the spine some time to start
sleep 2

# Start agent in the background
echo "🚀 Starting agent 'mpc_balancer'..."
$PIXI_CMD run upkie-mpc &
AGENT_PID=$!

# Wait for either process to exit
wait -n "$SIMULATION_PID" "$AGENT_PID"
EXIT_CODE=$?

# Clean up the remaining process
cleanup
