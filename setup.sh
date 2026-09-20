#!/usr/bin/env bash
set -euo pipefail

SIM_NUMBER=1
IFACE="arctic-sim-8-${SIM_NUMBER}"
DOWNLOADS="${HOME}/Downloads"
ZIP="${DOWNLOADS}/SIM-8-Wireguard-Configs.zip"
EXTRACTED="${DOWNLOADS}/SIM-8-Wireguard-Configs/SIM-8/${IFACE}.conf"
TMP_DIR="$(mktemp -d)"
trap 'rm -rf "${TMP_DIR}"' EXIT

die() { echo "Error: $*" >&2; exit 1; }

if [[ -f "${EXTRACTED}" ]]; then
    CONF="${EXTRACTED}"
elif [[ -f "${ZIP}" ]]; then
    CONF="${TMP_DIR}/${IFACE}.conf"
    unzip -p "${ZIP}" "SIM-8/${IFACE}.conf" > "${CONF}" ||
        die "${IFACE}.conf was not found in ${ZIP}"
else
    die "WireGuard config not found at ${EXTRACTED} or ${ZIP}"
fi

echo "=== WireGuard Setup for Arctic SIM-8 ==="

# Install wireguard if missing
if ! command -v wg-quick &>/dev/null; then
    echo "[1/3] Installing WireGuard..."
    if [[ "$(uname -s)" == "Darwin" ]]; then
        command -v brew >/dev/null || die "Homebrew is required to install WireGuard"
        brew install wireguard-tools
    else
        sudo apt update && sudo apt install -y wireguard
    fi
else
    echo "[1/3] WireGuard already installed."
fi

# Copy config
echo "[2/3] Copying tunnel config..."
sudo mkdir -p /etc/wireguard
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
