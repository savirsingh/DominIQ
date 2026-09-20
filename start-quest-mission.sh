#!/usr/bin/env bash
set -euo pipefail

MISSION_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_DIR="${MISSION_DIR}/../arctic-sim"
VENV="${MISSION_DIR}/.venv"
SIM_URL="http://localhost:8080"
SIM_API="http://localhost:8090"

die() { echo "Error: $*" >&2; exit 1; }

for tool in docker curl adb python3 pgrep; do
  command -v "$tool" >/dev/null || die "$tool is required"
done
if pgrep -f '[p]ython.*mission.py' >/dev/null; then
  die "mission.py is already running; stop it before starting another"
fi
[[ -f "${SIM_DIR}/docker-compose.yml" ]] || die "arctic-sim must be next to this repo"
[[ -f "${SIM_DIR}/sim/gzweb/gz3d/src/gzxr.js" ]] || die "arctic-sim needs the WebXR viewer branch"
[[ -f "${SIM_DIR}/.env" ]] || cp "${SIM_DIR}/.env.example" "${SIM_DIR}/.env"
docker info >/dev/null || die "start Docker Desktop first"
docker compose version >/dev/null || die "Docker Compose is required"
[[ "$(adb get-state 2>/dev/null || true)" == device ]] ||
  die "connect the Quest by USB and accept its USB debugging prompt, then rerun"

if [[ ! -x "${VENV}/bin/python" ]]; then
  python3 -m venv --system-site-packages "${VENV}"
fi
"${VENV}/bin/python" -m pip install --disable-pip-version-check -r "${MISSION_DIR}/requirements.txt"

echo "Starting the local Arctic simulator..."
(cd "${SIM_DIR}" && docker compose up -d --build)

echo "Waiting for the simulator and control API..."
ready=0
for ((attempt=0; attempt<120; attempt++)); do
  if curl -fsS --max-time 3 -o /dev/null "${SIM_URL}/" 2>/dev/null &&
     curl -fsS --max-time 3 -o /dev/null "${SIM_API}/api/assets" 2>/dev/null; then
    ready=1
    break
  fi
  sleep 2
done
((ready)) || die "simulator did not become ready; check docker compose logs in arctic-sim"
echo "Resetting local simulator..."
curl -fsS --max-time 10 -X POST "${SIM_API}/api/reset" >/dev/null ||
  die "could not reset local simulator"
echo "Waiting for vehicles, cameras, and Quest viewer to be ready..."
python3 - "${SIM_API}" "${SIM_URL}" <<'PY'
import json
import socket
import sys
import time
import urllib.request

api = sys.argv[1]
viewer = sys.argv[2]
names = {"quadcopter", "fixed-wing", "tower-1", "tower-2"}
deadline = time.monotonic() + 300
saw_reset = False
while time.monotonic() < deadline:
    try:
        with urllib.request.urlopen(f"{api}/api/status", timeout=5) as response:
            status = json.load(response)
        if status["state"] == "working":
            saw_reset = True
        if saw_reset and status["state"] == "idle" and status.get("detail", "").startswith("restart failed"):
            sys.exit(f"Simulator reset failed: {status['detail']}")
        if saw_reset and status["state"] == "idle" and status.get("detail", "").startswith("restarted"):
            with urllib.request.urlopen(f"{api}/api/assets", timeout=10) as response:
                assets = json.load(response)["assets"]
            ready = [asset for asset in assets if asset["name"] in names and asset["mavlink"] and asset["camera"]]
            if len(ready) == len(names):
                for asset in ready:
                    with socket.create_connection(("127.0.0.1", asset["cam"]), timeout=2):
                        pass
                with urllib.request.urlopen(viewer, timeout=5) as response:
                    if b"new GZ3D.WebXRView" in response.read():
                        break
    except (OSError, ValueError, KeyError):
        pass
    time.sleep(2)
else:
    sys.exit("Simulator or Quest viewer did not become ready within 5 minutes; check docker compose logs in arctic-sim")
PY

for port in 8080 8090 8600 8610 8630 8640; do
  adb reverse "tcp:${port}" "tcp:${port}"
done
adb shell am start -a android.intent.action.VIEW -d "${SIM_URL}" >/dev/null ||
  echo "Open ${SIM_URL} in Quest Browser manually."

echo "Quest: choose a vehicle for VR, or Tabletop map for passthrough AR."
echo "Starting mission; Ctrl+C resets the local simulator."
cd "${MISSION_DIR}"
trap 'curl -fsS --max-time 10 -X POST "${SIM_API}/api/reset" >/dev/null || true' EXIT
SIM_HOST=127.0.0.1 "${VENV}/bin/python" -u mission.py
