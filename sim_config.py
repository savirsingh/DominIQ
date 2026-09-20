"""Where the simulator is: the shared competition sim over WireGuard, or a local docker compose.

Every script in this repo gets its host, MAVLink ports and camera URLs from here, so switching
between the two is one flag or one environment variable:

    python3 mission.py --sim local
    SIM_TARGET=local python3 fly.py status

The default is "remote". Both targets expose the same ports (the local stack publishes the
same 14550 + 10*slot / 8600 + 10*slot layout); only the host and the tower positions differ.
"""
import os

# name -> MAVLink GCS UDP port, camera MJPEG port, vehicle type
ASSETS = {
    "quadcopter": {"udp": 14550, "cam": 8600, "type": "copter"},
    "fixed-wing": {"udp": 14560, "cam": 8610, "type": "plane"},
    "tower-1":    {"udp": 14580, "cam": 8630, "type": "tower"},
    "tower-2":    {"udp": 14590, "cam": 8640, "type": "tower"},
}

PROFILES = {
    "remote": {"host": "10.99.7.1"},
    "local":  {"host": "localhost"},
}

_active = "remote"


def select(name):
    """Choose the target. None keeps the current one (SIM_TARGET, else remote)."""
    global _active
    if name is None:
        return
    if name not in PROFILES:
        raise ValueError(f"unknown sim target {name!r}; choose from {', '.join(PROFILES)}")
    _active = name


def target():
    return _active


def is_local():
    return _active == "local"


def host():
    return PROFILES[_active]["host"]


def mavlink_url(asset):
    # The endpoints are MAVProxy udpin listeners, so the client must transmit first: udpout.
    return f"udpout:{host()}:{ASSETS[asset]['udp']}"


def camera_url(asset):
    return f"http://{host()}:{ASSETS[asset]['cam']}/stream"


def add_argument(parser):
    parser.add_argument(
        "--sim", choices=list(PROFILES), default=None,
        help="which simulator to talk to (default: $SIM_TARGET, else remote)")


def configure(args=None):
    """Apply --sim if given, otherwise $SIM_TARGET. Call once, right after parsing arguments."""
    select(getattr(args, "sim", None) or os.environ.get("SIM_TARGET") or None)


configure()
