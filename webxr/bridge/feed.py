#!/usr/bin/env python3
"""Serve asset GPS positions to the WebXR map as JSON.

    python3 bridge/feed.py --demo              # synthetic assets orbiting fort_ross; no sim needed
    python3 bridge/feed.py --sim local         # read MAVLink from the local simulator
    python3 bridge/feed.py --sim remote        # ...or the shared one over WireGuard

The headset page polls GET /positions (the vite dev server proxies /feed/positions to it):

    {"t": 1695000000.0, "assets": [
        {"name": "quadcopter", "kind": "copter", "lat": 71.99, "lon": -94.84, "alt": 155.8, "age": 0.1}, ...]}

`alt` is metres above mean sea level (GLOBAL_POSITION_INT.alt). The sim keeps that equal to world z,
and the terrain's sea level is z = 0, so the client plots it directly. `age` is seconds since the
asset was last heard from.

WITH THE MISSION RUNNING, do not use --sim: it would be a second MAVLink client on each asset's
port, and the sim's MAVProxy udpin listeners are only known to serve one (not tested). Let
mission.py serve this feed instead. It already holds the connections and the boat estimate:

    cd hack-the-north-2026 && git apply webxr/bridge/mission-feed.patch
    python3 mission.py --sim local --webxr-feed          # serves the same /positions on :8781

--sim here is for looking at the map without a mission.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

DEFAULT_PORT = 8781


class Feed:
    """Latest known fix per asset. Thread-safe; writers call update(), the HTTP handler calls snapshot()."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._assets: dict[str, dict] = {}

    def update(self, name: str, kind: str, lat: float, lon: float, alt: float) -> None:
        with self._lock:
            self._assets[name] = {"name": name, "kind": kind, "lat": lat, "lon": lon, "alt": alt,
                                  "_seen": time.monotonic()}

    def snapshot(self) -> dict:
        now = time.monotonic()
        with self._lock:
            assets = [{**{k: v for k, v in a.items() if k != "_seen"}, "age": round(now - a["_seen"], 2)}
                      for a in self._assets.values()]
        return {"t": time.time(), "assets": assets}


def serve(feed: Feed, port: int = DEFAULT_PORT, host: str = "0.0.0.0") -> ThreadingHTTPServer:
    """Start the HTTP server on a daemon thread and return it."""

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:  # noqa: N802
            if self.path.split("?", 1)[0] not in ("/positions", "/"):
                self.send_error(404)
                return
            body = json.dumps(feed.snapshot()).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *args) -> None:  # one line per poll would drown everything else
            pass

    # http.server sets SO_REUSEADDR, which lets macOS bind 0.0.0.0:<port> next to another program's
    # 127.0.0.1:<port>. Localhost then reaches that program instead of us. Fail loudly instead.
    ThreadingHTTPServer.allow_reuse_address = False
    server = ThreadingHTTPServer((host, port), Handler)
    threading.Thread(target=server.serve_forever, name="feed-http", daemon=True).start()
    return server


# ---------------------------------------------------------------------------
# Demo source: no simulator needed
# ---------------------------------------------------------------------------

# arctic-sim/out/fort_ross/sim.env FLEET: lat, lon, AMSL altitude of each asset's spawn.
DEMO_ANCHORS = {
    "quadcopter": ("copter", 71.9958070, -94.8393000, 75.79),
    "fixed-wing": ("plane", 71.9981950, -94.8419670, 0.0),
    "tower-1": ("tower", 71.9806710, -94.8537110, 115.65),
    "tower-2": ("tower", 72.0117780, -94.8047210, 225.36),
}


def offset(lat: float, lon: float, north_m: float, east_m: float) -> tuple[float, float]:
    """Small-offset lat/lon shift in true north/east metres. Fine for a few hundred metres."""
    return (lat + north_m / 111_132.0, lon + east_m / (111_320.0 * math.cos(math.radians(lat))))


def run_demo(feed: Feed) -> None:
    t0 = time.monotonic()
    plane = DEMO_ANCHORS["fixed-wing"]
    print("Demo: quadcopter circles its spawn, fixed-wing loops, boat drifts, towers stand still")
    while True:
        t = time.monotonic() - t0
        for name in ("tower-1", "tower-2"):
            kind, lat, lon, alt = DEMO_ANCHORS[name]
            feed.update(name, kind, lat, lon, alt)

        kind, lat, lon, alt = DEMO_ANCHORS["quadcopter"]
        a = t * 2 * math.pi / 60                                  # one lap a minute, r = 250 m
        la, lo = offset(lat, lon, 250 * math.sin(a), 250 * math.cos(a))
        feed.update("quadcopter", kind, la, lo, alt + 90 + 15 * math.sin(t / 5))

        a = -t * 2 * math.pi / 90                                 # one lap in 90 s, r = 700 m
        la, lo = offset(plane[1], plane[2], 700 * math.sin(a), 700 * math.cos(a) - 700)
        feed.update("fixed-wing", plane[0], la, lo, 220 + 20 * math.sin(t / 7))

        a = t * 2 * math.pi / 240                                 # slow loop on the water, r = 120 m
        la, lo = offset(plane[1], plane[2], 120 * math.sin(a), 120 * math.cos(a))
        feed.update("boat", "boat", la, lo, 0.0)
        time.sleep(0.1)


# ---------------------------------------------------------------------------
# Live source: pymavlink, same endpoints and idioms as mission.py
# ---------------------------------------------------------------------------

def run_mavlink(feed: Feed) -> None:
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))      # hack-the-north-2026/ for sim_config
    import sim_config
    from pymavlink import mavutil

    def watch(name: str) -> None:
        kind = sim_config.ASSETS[name]["type"]
        while True:
            try:
                conn = mavutil.mavlink_connection(sim_config.mavlink_url(name), source_system=255)
                last_beat = 0.0
                while True:
                    if time.monotonic() - last_beat > 1.0:
                        # A udpin listener only answers a client that has transmitted, so keep talking.
                        conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                        conn.mav.request_data_stream_send(0, 0, mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
                        last_beat = time.monotonic()
                    msg = conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
                    if msg is not None:
                        feed.update(name, kind, msg.lat / 1e7, msg.lon / 1e7, msg.alt / 1000.0)
            except Exception as exc:                                  # sim restarting, socket error, ...
                print(f"[{name}] {exc}; retrying in 2 s", file=sys.stderr)
                time.sleep(2)

    print(f"MAVLink from the {sim_config.target()} sim at {sim_config.host()}")
    print("Warning: mission.py must not be running against the same sim; see this file's docstring.")
    for name in sim_config.ASSETS:
        threading.Thread(target=watch, args=(name,), name=f"mav-{name}", daemon=True).start()
    while True:
        time.sleep(3600)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--demo", action="store_true", help="synthetic positions; no simulator needed")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--bind", default="0.0.0.0")
    known, _ = ap.parse_known_args()
    if not known.demo:
        sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
        import sim_config
        sim_config.add_argument(ap)
    args = ap.parse_args()
    if not args.demo:
        import sim_config
        sim_config.configure(args)

    feed = Feed()
    serve(feed, args.port, args.bind)
    print(f"Serving http://{args.bind}:{args.port}/positions")
    try:
        (run_demo if args.demo else run_mavlink)(feed)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
