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

## Usage

### Local simulator and Quest

With `arctic-sim` checked out beside this repo, Docker Desktop running, and a
Quest connected by USB with debugging allowed, run:

```bash
./start-quest-mission.sh
```

The script builds and starts the local simulator, forwards its browser and
camera ports to Quest, opens `http://localhost:8080` there, then runs the
mission against the local simulator. Choose a vehicle for VR or **Tabletop map
(AR)** for a passthrough terrain map with live asset pointers. Ctrl+C stops the
mission and resets the local simulator. The first confirmed boat detection
briefly appears in either Quest view; the vessel remains a map marker rather
than a camera destination.

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
