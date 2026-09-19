#!/usr/bin/env bash
set -euo pipefail

CONF="/home/savir/Downloads/SIM-8-Wireguard-Configs/SIM-8/arctic-sim-8-1.conf"
IFACE="arctic-sim-8-1"

echo "=== WireGuard Setup for Arctic SIM-8 ==="

# Install wireguard if missing
if ! command -v wg &>/dev/null; then
    echo "[1/3] Installing WireGuard..."
    sudo apt update && sudo apt install -y wireguard
else
    echo "[1/3] WireGuard already installed."
fi

# Copy config
echo "[2/3] Copying tunnel config..."
sudo cp "$CONF" /etc/wireguard/"$IFACE".conf
sudo chmod 600 /etc/wireguard/"$IFACE".conf

# Bring up tunnel (tear down first if already active)
echo "[3/3] Activating tunnel..."
sudo wg-quick down "$IFACE" 2>/dev/null || true
sudo wg-quick up "$IFACE"

echo ""
echo "=== Tunnel active ==="
sudo wg show
echo ""
echo "Done! Open your SIM URL in Chrome to access the scene."
