# Arctic SIM-8 — Boat Tracker

Find and track the target ship using drones (quadcopter, fixed-wing) and two towers.

## Setup

### 1. WireGuard VPN

```bash
# Ubuntu
sudo apt install wireguard
sudo cp SIM-8/arctic-sim-8-1.conf /etc/wireguard/arctic-sim-8-1.conf
sudo chmod 600 /etc/wireguard/arctic-sim-8-1.conf
sudo wg-quick up arctic-sim-8-1
```

Or run `bash setup.sh` (does all the above).

Each teammate needs a **different** config file (arctic-sim-8-1.conf, arctic-sim-8-2.conf, etc.) — they're in the shared Google Drive folder (not committed here since they contain private keys).

### 2. Install dependencies

```bash
pip install pymavlink
```

### 3. Verify connection

Open http://10.99.7.1:8080 in Chrome — you should see the 3D sim.

## Remote or local sim

Everything here runs against either the shared sim (over WireGuard, the default) or your own
`arctic-sim` docker compose stack. Pick one with a flag or an environment variable:

```bash
python3 mission.py                    # shared sim at 10.99.7.1 (default)
python3 mission.py --sim local        # your own sim at localhost
export SIM_TARGET=local               # or set it once for the whole shell
```

`--sim` works on `mission.py`, `fly.py`, `tracker.py --live` and `tower_watch.py`; for the
live map open `map.html?sim=local`. Host, ports and camera URLs all live in `sim_config.py`.

The two sims use the same ports, but the local one places its towers from its own `.env`, so
they are not where the shared sim has them. In local mode `mission.py` reads each tower's
position from its telemetry at startup instead of using the shared-sim coordinates, so the
quadcopter's search corridor and the fixed-wing's tower-rush target follow your towers. It
stops with an error if the local sim isn't reachable (check with `./mavcheck` in `arctic-sim`).
`map.html?sim=local` uses the tower positions from the default `.env` and does not track edits.

## Tower sweep

In `mission.py` each tower cycles through areas along the horizon (the logic of `tower_aim.py`
in anomaly-demo): hold still and learn the background for 10 s, watch for movement for 20 s,
then turn 50 degrees to a new area and start again. At the end of its pan range a tower reverses,
so it keeps covering the whole horizon. The timings and step are `TOWER_LEARN_S`, `TOWER_WATCH_S`
and `TOWER_STEP_DEG` at the top of `mission.py`. Detections are saved to `tower_output/`.

## Usage

### Check asset status

```bash
python3 fly.py status
```

### Fly the quadcopter

```bash
python3 fly.py takeoff quadcopter 50       # arm + takeoff to 50m
python3 fly.py goto quadcopter LAT LON ALT  # fly to coordinates
python3 fly.py land quadcopter              # land
```

### Track the boat

```bash
python3 tracker.py          # test mode (synthetic coordinates)
python3 tracker.py --live   # live mode (reads drone telemetry from SIM)
```

The tracker estimates the boat's GPS position from each drone's:
- GPS coordinates (lat, lon)
- Altitude (AGL)
- Camera look-down angle (elevation from horizontal)
- Camera heading

Math: `horizontal_distance = altitude / tan(elevation_angle)`, projected along the heading.

## SIM Assets

| Name        | Type   | UDP Port | IP (internal) |
|-------------|--------|----------|---------------|
| quadcopter  | copter | 14550    | 10.23.0.100   |
| fixed-wing  | plane  | 14560    | 10.23.0.101   |
| tower-1     | tower  | 14580    | 10.23.0.103   |
| tower-2     | tower  | 14590    | 10.23.0.104   |

All assets are reached via MAVLink UDP at `10.99.7.1:<port>`.

## API

The SIM control panel runs at `http://10.99.7.1:8090`:
- `GET /api/assets` — asset list + MAVLink status
- `GET /api/status` — sim state
- `GET /api/site` — geo reference (bounds, centre lat/lon)
- `POST /api/reset` — reset the sim
