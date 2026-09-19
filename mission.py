#!/usr/bin/env python3
"""
Coordinated drone mission controller for Arctic SIM-8.

Orchestrates all 4 assets:
  - Quadcopter + fixed-wing: take off and circle on patrol
  - Tower-1 + tower-2: stay still, watch for movement
  - When a tower detects motion → fixed-wing rushes to that area
  - When the boat is found → fixed-wing locks on and follows it

Usage:
    python3 mission.py

Press Ctrl+C to land all drones and shut down.
"""

from pymavlink import mavutil
from tracker import estimate_boat_position, haversine
import threading
import math
import time
import sys

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

SIM_HOST = "10.99.7.1"

ASSETS = {
    "quadcopter": {"udp": 14550, "type": "copter", "guided_mode": 4},
    "fixed-wing": {"udp": 14560, "type": "plane",  "guided_mode": 15},
    "tower-1":    {"udp": 14580, "type": "tower",  "guided_mode": None},
    "tower-2":    {"udp": 14590, "type": "tower",  "guided_mode": None},
}

SITE_CENTER = (71.99196, -94.822428)

TOWER_POSITIONS = {
    "tower-1": (71.980671, -94.853711),
    "tower-2": (72.011778, -94.804721),
}

QUAD_PATROL_RADIUS_M = 800
QUAD_PATROL_ALT = 60
QUAD_PATROL_WAYPOINTS = 12

PLANE_PATROL_RADIUS_M = 1200
PLANE_PATROL_ALT = 80
PLANE_PATROL_WAYPOINTS = 12

WAYPOINT_ARRIVAL_THRESHOLD_M = 30

EARTH_RADIUS = 6_371_000

# ---------------------------------------------------------------------------
# Shared state — teammate integration points are the booleans here
# ---------------------------------------------------------------------------

class SharedState:
    def __init__(self):
        self.lock = threading.Lock()

        # === TEAMMATE INTEGRATION POINT ===
        # Set these to True from your OpenCV motion-detection code.
        # Camera feeds:
        #   tower-1: http://10.99.7.1:8630
        #   tower-2: http://10.99.7.1:8640
        self.tower1_detected = False
        self.tower2_detected = False

        # === TEAMMATE INTEGRATION POINT ===
        # Set boat_found = True when your AI model identifies the boat
        # in a drone camera frame. Camera feeds:
        #   quadcopter: http://10.99.7.1:8600
        #   fixed-wing: http://10.99.7.1:8610
        self.boat_found = False
        self.boat_lat = 0.0
        self.boat_lon = 0.0

        self.locked = False
        self.shutdown = False

        self.asset_positions = {}

# ---------------------------------------------------------------------------
# MAVLink helpers
# ---------------------------------------------------------------------------

def connect(name):
    info = ASSETS[name]
    addr = f"udpout:{SIM_HOST}:{info['udp']}"
    conn = mavutil.mavlink_connection(addr, source_system=255)
    for _ in range(5):
        conn.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        time.sleep(0.2)
    conn.mav.request_data_stream_send(
        0, 0, mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
    msg = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
    if not msg:
        raise ConnectionError(f"No heartbeat from {name}")
    return conn


def heartbeat_loop(conn, state):
    while not state.shutdown:
        try:
            conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        except Exception:
            break
        time.sleep(1)


def set_mode(conn, mode_id):
    conn.mav.set_mode_send(
        conn.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)
    time.sleep(1)


def arm(conn):
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0)
    msg = conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
    return msg and msg.result == 0


def takeoff_and_wait(conn, alt, state, timeout=40):
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, alt)
    start = time.time()
    while time.time() - start < timeout and not state.shutdown:
        gps = conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
        if gps and gps.relative_alt / 1e3 >= alt * 0.85:
            return True
        time.sleep(0.5)
    return False


def get_position(conn):
    gps = conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
    if not gps:
        return None
    return (gps.lat / 1e7, gps.lon / 1e7, gps.relative_alt / 1e3)


def send_goto(conn, lat, lon, alt):
    conn.mav.set_position_target_global_int_send(
        0, conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,
        int(lat * 1e7), int(lon * 1e7), alt,
        0, 0, 0,
        0, 0, 0,
        0, 0)


def land(conn):
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0)


def dist_between(lat1, lon1, lat2, lon2):
    dlat = (lat2 - lat1) * 111320
    dlon = (lon2 - lon1) * 111320 * math.cos(math.radians(lat1))
    return math.sqrt(dlat ** 2 + dlon ** 2)

# ---------------------------------------------------------------------------
# Waypoint generation
# ---------------------------------------------------------------------------

def generate_circle_waypoints(center_lat, center_lon, radius_m, n):
    waypoints = []
    for i in range(n):
        bearing = 2 * math.pi * i / n
        dlat = radius_m * math.cos(bearing) / 111320
        dlon = radius_m * math.sin(bearing) / (111320 * math.cos(math.radians(center_lat)))
        waypoints.append((center_lat + dlat, center_lon + dlon))
    return waypoints

# ---------------------------------------------------------------------------
# Asset threads
# ---------------------------------------------------------------------------

def run_quadcopter(state):
    name = "quadcopter"
    info = ASSETS[name]
    try:
        log(name, "connecting...")
        conn = connect(name)
        threading.Thread(target=heartbeat_loop, args=(conn, state), daemon=True).start()

        log(name, "setting GUIDED mode")
        set_mode(conn, info["guided_mode"])

        log(name, "arming")
        if not arm(conn):
            log(name, "ARM FAILED")
            return
        time.sleep(1)

        log(name, f"taking off to {QUAD_PATROL_ALT}m")
        if not takeoff_and_wait(conn, QUAD_PATROL_ALT, state):
            log(name, "takeoff timed out, continuing anyway")

        waypoints = generate_circle_waypoints(
            *SITE_CENTER, QUAD_PATROL_RADIUS_M, QUAD_PATROL_WAYPOINTS)
        wp_idx = 0

        log(name, f"patrol started — {len(waypoints)} waypoints, {QUAD_PATROL_RADIUS_M}m radius")

        while not state.shutdown:
            if state.boat_found:
                log(name, "boat found — hovering at current position")
                while state.boat_found and not state.shutdown:
                    pos = get_position(conn)
                    if pos:
                        with state.lock:
                            state.asset_positions[name] = pos
                    time.sleep(2)
                continue

            wlat, wlon = waypoints[wp_idx]
            send_goto(conn, wlat, wlon, QUAD_PATROL_ALT)

            while not state.shutdown and not state.boat_found:
                pos = get_position(conn)
                if not pos:
                    time.sleep(0.5)
                    continue
                with state.lock:
                    state.asset_positions[name] = pos
                d = dist_between(pos[0], pos[1], wlat, wlon)
                if d < WAYPOINT_ARRIVAL_THRESHOLD_M:
                    break
                time.sleep(1)

            wp_idx = (wp_idx + 1) % len(waypoints)

    except Exception as e:
        log(name, f"error: {e}")
    finally:
        try:
            log(name, "landing")
            land(conn)
        except Exception:
            pass


def run_fixed_wing(state):
    name = "fixed-wing"
    info = ASSETS[name]
    try:
        log(name, "connecting...")
        conn = connect(name)
        threading.Thread(target=heartbeat_loop, args=(conn, state), daemon=True).start()

        # ArduPlane needs TAKEOFF mode (13) to launch, then switch to GUIDED (15)
        log(name, "setting TAKEOFF mode")
        set_mode(conn, 13)

        conn.mav.command_long_send(
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, PLANE_PATROL_ALT)
        time.sleep(0.5)

        log(name, "arming")
        conn.mav.command_long_send(
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0)
        ack = conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
        if not (ack and ack.result == 0):
            log(name, f"ARM FAILED: {ack}")
            return
        time.sleep(1)

        log(name, f"taking off to {PLANE_PATROL_ALT}m")
        if not takeoff_and_wait(conn, PLANE_PATROL_ALT, state, timeout=60):
            log(name, "takeoff timed out, continuing anyway")

        log(name, "switching to GUIDED mode for patrol")
        set_mode(conn, 15)

        waypoints = generate_circle_waypoints(
            *SITE_CENTER, PLANE_PATROL_RADIUS_M, PLANE_PATROL_WAYPOINTS)
        wp_idx = 0

        log(name, f"patrol started — {len(waypoints)} waypoints, {PLANE_PATROL_RADIUS_M}m radius")

        while not state.shutdown:
            # --- Phase 3: boat found → lock on and follow ---
            if state.boat_found:
                log(name, "BOAT FOUND — locking on")
                state.locked = True
                while state.boat_found and not state.shutdown:
                    with state.lock:
                        blat, blon = state.boat_lat, state.boat_lon
                    if blat != 0.0 and blon != 0.0:
                        send_goto(conn, blat, blon, PLANE_PATROL_ALT)
                    pos = get_position(conn)
                    if pos:
                        with state.lock:
                            state.asset_positions[name] = pos
                    time.sleep(1.5)
                continue

            # --- Phase 2: tower detected motion → rush to that area ---
            if state.tower1_detected or state.tower2_detected:
                if state.tower1_detected:
                    target = TOWER_POSITIONS["tower-1"]
                    tname = "tower-1"
                else:
                    target = TOWER_POSITIONS["tower-2"]
                    tname = "tower-2"

                log(name, f"TOWER {tname} detected motion — rushing to area")
                search_wps = generate_circle_waypoints(
                    target[0], target[1], 400, 8)
                search_idx = 0

                while (state.tower1_detected or state.tower2_detected) \
                        and not state.boat_found and not state.shutdown:
                    slat, slon = search_wps[search_idx]
                    send_goto(conn, slat, slon, PLANE_PATROL_ALT)

                    while not state.shutdown and not state.boat_found:
                        pos = get_position(conn)
                        if not pos:
                            time.sleep(0.5)
                            continue
                        with state.lock:
                            state.asset_positions[name] = pos
                        d = dist_between(pos[0], pos[1], slat, slon)
                        if d < WAYPOINT_ARRIVAL_THRESHOLD_M:
                            break
                        time.sleep(1)

                    search_idx = (search_idx + 1) % len(search_wps)
                continue

            # --- Phase 1: patrol circle ---
            wlat, wlon = waypoints[wp_idx]
            send_goto(conn, wlat, wlon, PLANE_PATROL_ALT)

            while not state.shutdown and not state.boat_found \
                    and not state.tower1_detected and not state.tower2_detected:
                pos = get_position(conn)
                if not pos:
                    time.sleep(0.5)
                    continue
                with state.lock:
                    state.asset_positions[name] = pos
                d = dist_between(pos[0], pos[1], wlat, wlon)
                if d < WAYPOINT_ARRIVAL_THRESHOLD_M:
                    break
                time.sleep(1)

            wp_idx = (wp_idx + 1) % len(waypoints)

    except Exception as e:
        log(name, f"error: {e}")
    finally:
        try:
            log(name, "landing")
            land(conn)
        except Exception:
            pass


def run_tower(name, state):
    try:
        log(name, "connecting...")
        conn = connect(name)
        threading.Thread(target=heartbeat_loop, args=(conn, state), daemon=True).start()

        log(name, "online — monitoring")

        while not state.shutdown:
            pos = get_position(conn)
            if pos:
                with state.lock:
                    state.asset_positions[name] = pos
            time.sleep(3)

            # === TEAMMATE INTEGRATION POINT ===
            # Replace this stub with your OpenCV motion detection logic.
            # When motion is detected on this tower's camera feed:
            #
            #   if name == "tower-1":
            #       state.tower1_detected = True
            #   elif name == "tower-2":
            #       state.tower2_detected = True
            #
            # Camera feeds:
            #   tower-1: http://10.99.7.1:8630
            #   tower-2: http://10.99.7.1:8640

    except Exception as e:
        log(name, f"error: {e}")


def run_tracker(state):
    """
    When boat_found is True, continuously update state.boat_lat/boat_lon
    from the fixed-wing's telemetry using the position estimator.
    """
    try:
        conn = connect("fixed-wing")
        threading.Thread(target=heartbeat_loop, args=(conn, state), daemon=True).start()

        while not state.shutdown:
            if not state.boat_found:
                time.sleep(1)
                continue

            gps = conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
            att = conn.recv_match(type="ATTITUDE", blocking=True, timeout=1)

            if gps and att:
                lat = gps.lat / 1e7
                lon = gps.lon / 1e7
                alt = gps.relative_alt / 1e3

                if alt < 1:
                    time.sleep(1)
                    continue

                pitch_deg = math.degrees(att.pitch)
                yaw_deg = math.degrees(att.yaw)
                camera_elevation = max(0.1, min(90, 90 + pitch_deg))
                camera_heading = yaw_deg % 360

                try:
                    est_lat, est_lon, est_dist = estimate_boat_position(
                        lat, lon, alt, camera_elevation, camera_heading)
                    with state.lock:
                        state.boat_lat = est_lat
                        state.boat_lon = est_lon
                        state.locked = True
                except ValueError:
                    pass

            time.sleep(1)

    except Exception as e:
        log("tracker", f"error: {e}")

# ---------------------------------------------------------------------------
# Logging + status
# ---------------------------------------------------------------------------

_log_lock = threading.Lock()

def log(source, msg):
    with _log_lock:
        t = time.strftime("%H:%M:%S")
        print(f"[{t}] [{source:12s}] {msg}")


def status_printer(state):
    while not state.shutdown:
        time.sleep(10)
        with state.lock:
            positions = dict(state.asset_positions)
        lines = []
        for name in ["quadcopter", "fixed-wing", "tower-1", "tower-2"]:
            pos = positions.get(name)
            if pos:
                lines.append(f"  {name:12s}  ({pos[0]:.5f}, {pos[1]:.5f})  alt={pos[2]:5.1f}m")
            else:
                lines.append(f"  {name:12s}  no data")

        flags = []
        if state.tower1_detected:
            flags.append("tower-1:DETECTED")
        if state.tower2_detected:
            flags.append("tower-2:DETECTED")
        if state.boat_found:
            flags.append(f"BOAT({state.boat_lat:.5f},{state.boat_lon:.5f})")
        if state.locked:
            flags.append("LOCKED")

        with _log_lock:
            print(f"\n[{time.strftime('%H:%M:%S')}] --- STATUS ---")
            for l in lines:
                print(l)
            if flags:
                print(f"  flags: {', '.join(flags)}")
            print()

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    print("=" * 60)
    print("ARCTIC SIM-8 — COORDINATED MISSION")
    print("=" * 60)
    print()
    print("Assets: quadcopter, fixed-wing, tower-1, tower-2")
    print("Ctrl+C to land all drones and exit")
    print()

    state = SharedState()

    threads = [
        threading.Thread(target=run_quadcopter, args=(state,), name="quadcopter"),
        threading.Thread(target=run_fixed_wing, args=(state,), name="fixed-wing"),
        threading.Thread(target=run_tower, args=("tower-1", state), name="tower-1"),
        threading.Thread(target=run_tower, args=("tower-2", state), name="tower-2"),
        threading.Thread(target=run_tracker, args=(state,), name="tracker"),
        threading.Thread(target=status_printer, args=(state,), name="status"),
    ]

    for t in threads:
        t.daemon = True
        t.start()

    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n\nShutting down — landing drones...")
        state.shutdown = True
        for t in threads:
            t.join(timeout=10)
        print("Done.")


if __name__ == "__main__":
    main()
