#!/usr/bin/env bash
set -euo pipefail

MISSION_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_DIR="${MISSION_DIR}/../arctic-sim"
VENV="${MISSION_DIR}/.venv"
SIM_URL="http://localhost:8080"

die() { echo "Error: $*" >&2; exit 1; }

for tool in docker curl adb python3; do
  command -v "$tool" >/dev/null || die "$tool is required"
done
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
     curl -fsS --max-time 3 -o /dev/null "http://localhost:8090/api/assets" 2>/dev/null; then
    ready=1
    break
  fi
  sleep 2
done
((ready)) || die "simulator did not become ready; check docker compose logs in arctic-sim"
page="$(curl -fsS --max-time 5 "${SIM_URL}/")"
[[ "${page}" == *'new GZ3D.WebXRView'* ]] || die "localhost:8080 is not serving the WebXR viewer"

for port in 8080 8090 8600 8610 8630 8640; do
  adb reverse "tcp:${port}" "tcp:${port}"
done
adb shell am start -a android.intent.action.VIEW -d "${SIM_URL}" >/dev/null ||
  echo "Open ${SIM_URL} in Quest Browser manually."

echo "Quest: choose a vehicle for VR, or Tabletop map for passthrough AR."
echo "Starting mission; Ctrl+C stops it and resets the local simulator."
cd "${MISSION_DIR}"
trap 'curl -fsS --max-time 10 -X POST http://localhost:8090/api/reset >/dev/null || true' EXIT
SIM_HOST=127.0.0.1 "${VENV}/bin/python" -u mission.py
