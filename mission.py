#!/usr/bin/env python3
"""
Coordinated drone mission controller for Arctic SIM-8.

All 4 assets patrol and scan for the boat using YOLO. When any camera
spots the boat, its GPS coords are estimated from the camera geometry,
and both drones converge on it.

Usage:  python3 mission.py
Stop:   Ctrl+C (lands both drones)
"""

from pymavlink import mavutil
from tracker import offset_lat_lon
import threading
import math
import time
import sys
import os
import cv2
import numpy as np

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

SIM_HOST = "10.99.7.1"

ASSETS = {
    "quadcopter": {"udp": 14550, "guided_mode": 4,  "is_plane": False},
    "fixed-wing": {"udp": 14560, "guided_mode": 15, "is_plane": True},
}

TOWER_CAMS = {
    "tower-1": {"udp": 14580, "cam": f"http://{SIM_HOST}:8630/stream",
                "width": 1280, "height": 720, "hfov": 1.047},
    "tower-2": {"udp": 14590, "cam": f"http://{SIM_HOST}:8640/stream",
                "width": 1280, "height": 720, "hfov": 1.047},
}

DRONE_CAMS = {
    "quadcopter": {"cam": f"http://{SIM_HOST}:8600/stream",
                   "width": 960, "height": 720, "hfov": 2.0, "mount_pitch": 0.0},
    "fixed-wing": {"cam": f"http://{SIM_HOST}:8610/stream",
                   "width": 1280, "height": 720, "hfov": 1.204, "mount_pitch": 0.14},
}

SITE_CENTER = (71.99196, -94.822428)

QUAD_RADIUS = 1800
QUAD_ALT = 60
PLANE_RADIUS = 2800
PLANE_ALT = 80
N_WAYPOINTS = 16
ARRIVAL_M = 30
ORBIT_RADIUS = 400    # metres — orbit the boat at this distance once found
ORBIT_POINTS = 8      # waypoints in the orbit circle
GOTO_INTERVAL = 5     # seconds — only resend goto when target moves this much

MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "best.pt")
MODEL_CONF = 0.25

# ---------------------------------------------------------------------------
# Shared state
# ---------------------------------------------------------------------------

class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.boat_found = False
        self.boat_lat = 0.0
        self.boat_lon = 0.0
        self.shutdown = False
        self.positions = {}   # name → (lat, lon, alt)
        self.attitudes = {}   # name → (heading_deg, pitch_deg)

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------

_log_lock = threading.Lock()

def log(src, msg):
    with _log_lock:
        print(f"[{time.strftime('%H:%M:%S')}] [{src:12s}] {msg}")

# ---------------------------------------------------------------------------
# YOLO detector
# ---------------------------------------------------------------------------

_detector = None

def get_detector():
    global _detector
    if _detector is None:
        try:
            from ultralytics import YOLO
            if os.path.exists(MODEL_PATH):
                _detector = YOLO(MODEL_PATH)
                log("detector", f"loaded {MODEL_PATH}")
            else:
                log("detector", f"model not found: {MODEL_PATH}")
        except ImportError:
            log("detector", "ultralytics not installed")
    return _detector


def yolo_detect(img, conf=MODEL_CONF):
    model = get_detector()
    if model is None:
        return []
    results = model(img, conf=conf, verbose=False)
    dets = []
    for r in results:
        for box in r.boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            c = float(box.conf[0])
            dets.append((c, int(x1), int(y1), int(x2 - x1), int(y2 - y1)))
    return dets

# ---------------------------------------------------------------------------
# Camera geometry: bbox pixel → boat GPS
# ---------------------------------------------------------------------------

def bbox_to_gps(cx_px, cy_px, drone_lat, drone_lon, drone_alt,
                heading_deg, pitch_deg, cam):
    """Convert a detection's center pixel to a GPS coordinate."""
    if drone_alt < 2:
        return None
    focal = (cam["width"] / 2) / math.tan(cam["hfov"] / 2)
    az = math.atan2(cx_px - cam["width"] / 2, focal)
    el = math.atan2(cam["height"] / 2 - cy_px, focal)
    mount = cam.get("mount_pitch", 0.0)
    look_down = -(math.radians(pitch_deg) - mount + el)
    if look_down < 0.02:
        return None
    dist = drone_alt / math.tan(look_down)
    if dist < 0 or dist > 5000:
        return None
    bearing = math.radians(heading_deg) + az
    return offset_lat_lon(drone_lat, drone_lon, dist, bearing)

# ---------------------------------------------------------------------------
# Camera frame grabber (non-blocking, runs in a thread)
# ---------------------------------------------------------------------------

class Cam:
    def __init__(self, url):
        self.url = url
        self._frame = None
        self._lock = threading.Lock()
        t = threading.Thread(target=self._loop, daemon=True)
        t.start()

    def _loop(self):
        while True:
            try:
                cap = cv2.VideoCapture(self.url)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                cap.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 5000)
                cap.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, 5000)
                while cap.isOpened():
                    ok, frame = cap.read()
                    if not ok:
                        break
                    with self._lock:
                        self._frame = frame
                cap.release()
            except Exception:
                pass
            time.sleep(0.5)

    def grab(self):
        with self._lock:
            return self._frame.copy() if self._frame is not None else None

# ---------------------------------------------------------------------------
# MAVLink helpers
# ---------------------------------------------------------------------------

def mav_connect(name, udp_port):
    addr = f"udpout:{SIM_HOST}:{udp_port}"
    conn = mavutil.mavlink_connection(addr, source_system=255)
    for _ in range(5):
        conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        time.sleep(0.2)
    conn.mav.request_data_stream_send(0, 0, mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
    conn.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
    return conn


def wait_for_ekf(name, conn):
    """Wait for EKF to be healthy. Times out after 30s."""
    log(name, "waiting for EKF...")
    t0 = time.time()
    while time.time() - t0 < 30:
        conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        m = conn.recv_match(type="EKF_STATUS_REPORT", blocking=True, timeout=1)
        if m and m.flags & 0x1F == 0x1F:
            log(name, f"EKF ready ({time.time()-t0:.0f}s)")
            return True
    log(name, "EKF wait timed out, proceeding anyway")
    return False


def mav_heartbeat(conn, state):
    while not state.shutdown:
        try:
            conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                    mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        except Exception:
            break
        time.sleep(1)


def mav_set_mode(conn, mode):
    conn.mav.set_mode_send(conn.target_system,
                           mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode)
    t0 = time.time()
    while time.time() - t0 < 3:
        conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        m = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if m and m.custom_mode == mode:
            return True
    return False


def mav_arm(conn, name):
    for _ in range(5):
        conn.mav.command_long_send(conn.target_system, conn.target_component,
                                   mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                   0, 1, 0, 0, 0, 0, 0, 0)
        t0 = time.time()
        while time.time() - t0 < 4:
            conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                    mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            m = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if m and m.base_mode & 128:
                return True
        time.sleep(1)
    return False


def mav_takeoff(conn, alt, state, timeout=60):
    conn.mav.command_long_send(conn.target_system, conn.target_component,
                               mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                               0, 0, 0, 0, 0, 0, 0, alt)
    t0 = time.time()
    while time.time() - t0 < timeout and not state.shutdown:
        conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        m = conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
        if m and m.relative_alt / 1e3 >= alt * 0.8:
            return True
        time.sleep(0.5)
    return False


def mav_goto(conn, lat, lon, alt):
    conn.mav.set_position_target_global_int_send(
        0, conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,
        int(lat * 1e7), int(lon * 1e7), alt,
        0, 0, 0, 0, 0, 0, 0, 0)


def mav_land(conn):
    conn.mav.command_long_send(conn.target_system, conn.target_component,
                               mavutil.mavlink.MAV_CMD_NAV_LAND,
                               0, 0, 0, 0, 0, 0, 0, 0)


def mav_read(conn):
    """Read GPS + attitude in one pass. Returns (lat, lon, alt, hdg, pitch) or None."""
    gps = att = None
    t0 = time.time()
    while time.time() - t0 < 1.5:
        m = conn.recv_match(blocking=True, timeout=0.3)
        if m is None:
            continue
        t = m.get_type()
        if t == "GLOBAL_POSITION_INT":
            gps = m
        elif t == "ATTITUDE":
            att = m
        if gps:
            break
    if not gps:
        return None
    lat = gps.lat / 1e7
    lon = gps.lon / 1e7
    alt = gps.relative_alt / 1e3
    hdg = gps.hdg / 100.0
    pitch = 0.0
    if att:
        hdg = math.degrees(att.yaw) % 360
        pitch = math.degrees(att.pitch)
    return lat, lon, alt, hdg, pitch

# ---------------------------------------------------------------------------
# Waypoint circle
# ---------------------------------------------------------------------------

def make_circle(lat, lon, radius_m, n):
    wps = []
    for i in range(n):
        a = 2 * math.pi * i / n
        dlat = radius_m * math.cos(a) / 111320
        dlon = radius_m * math.sin(a) / (111320 * math.cos(math.radians(lat)))
        wps.append((lat + dlat, lon + dlon))
    return wps


def dist_m(lat1, lon1, lat2, lon2):
    dlat = (lat2 - lat1) * 111320
    dlon = (lon2 - lon1) * 111320 * math.cos(math.radians(lat1))
    return math.sqrt(dlat ** 2 + dlon ** 2)

# ---------------------------------------------------------------------------
# Detection scanner — runs YOLO in a dedicated thread, never blocks flight
# ---------------------------------------------------------------------------

def run_scanner(name, cam_url, cam_info, state):
    """Grab frames, run YOLO, always update boat position when visible."""
    cam = Cam(cam_url)
    time.sleep(5)
    log(name + "-scan", "scanning started")
    while not state.shutdown:
        frame = cam.grab()
        if frame is None:
            time.sleep(1)
            continue

        with state.lock:
            pos = state.positions.get(name)
            att = state.attitudes.get(name)
        if not pos or not att:
            time.sleep(1)
            continue

        dets = yolo_detect(frame)
        if dets:
            conf, x, y, w, h = dets[0]
            coords = bbox_to_gps(x + w/2, y + h/2,
                                 pos[0], pos[1], pos[2],
                                 att[0], att[1], cam_info)
            if coords:
                with state.lock:
                    if not state.boat_found:
                        log(name + "-scan", f"BOAT DETECTED! conf={conf:.2f} → ({coords[0]:.6f}, {coords[1]:.6f})")
                        state.boat_found = True
                    state.boat_lat = coords[0]
                    state.boat_lon = coords[1]

        time.sleep(1.5)

# ---------------------------------------------------------------------------
# Drone flight threads
# ---------------------------------------------------------------------------

def fly_copter(state):
    name = "quadcopter"
    info = ASSETS[name]
    cam_info = DRONE_CAMS[name]
    conn = None
    try:
        log(name, "connecting...")
        conn = mav_connect(name, info["udp"])
        wait_for_ekf(name, conn)

        log(name, "GUIDED mode")
        mav_set_mode(conn, info["guided_mode"])

        log(name, "arming")
        if not mav_arm(conn, name):
            log(name, "ARM FAILED"); return

        log(name, f"takeoff → {QUAD_ALT}m")
        if mav_takeoff(conn, QUAD_ALT, state):
            log(name, "airborne")
        else:
            log(name, "takeoff slow, continuing")

        threading.Thread(target=mav_heartbeat, args=(conn, state), daemon=True).start()
        threading.Thread(target=run_scanner, args=(name, cam_info["cam"], cam_info, state), daemon=True).start()

        wps = make_circle(*SITE_CENTER, QUAD_RADIUS, N_WAYPOINTS)
        idx = 0
        log(name, f"patrol — {len(wps)} waypoints, {QUAD_RADIUS}m radius")

        orbit_idx = 0
        orbit_wps = []
        last_orbit_center = (0, 0)
        last_goto_time = 0

        while not state.shutdown:
            t = mav_read(conn)
            if t:
                with state.lock:
                    state.positions[name] = t[:3]
                    state.attitudes[name] = t[3:]

            if state.boat_found:
                with state.lock:
                    blat, blon = state.boat_lat, state.boat_lon

                # Rebuild orbit if boat moved >50m
                if dist_m(blat, blon, *last_orbit_center) > 50:
                    orbit_wps = make_circle(blat, blon, ORBIT_RADIUS, ORBIT_POINTS)
                    last_orbit_center = (blat, blon)
                    orbit_idx = 0

                if orbit_wps:
                    owlat, owlon = orbit_wps[orbit_idx]
                    # Only resend goto every GOTO_INTERVAL seconds
                    if time.time() - last_goto_time > GOTO_INTERVAL:
                        mav_goto(conn, owlat, owlon, QUAD_ALT)
                        last_goto_time = time.time()
                    if t and dist_m(t[0], t[1], owlat, owlon) < ARRIVAL_M:
                        orbit_idx = (orbit_idx + 1) % len(orbit_wps)
                        last_goto_time = 0  # force new goto
                time.sleep(1)
                continue

            # Patrol
            wlat, wlon = wps[idx]
            mav_goto(conn, wlat, wlon, QUAD_ALT)

            for _ in range(30):
                if state.shutdown or state.boat_found:
                    break
                t = mav_read(conn)
                if t:
                    with state.lock:
                        state.positions[name] = t[:3]
                        state.attitudes[name] = t[3:]
                    if dist_m(t[0], t[1], wlat, wlon) < ARRIVAL_M:
                        break
                time.sleep(1)

            idx = (idx + 1) % len(wps)

    except Exception as e:
        log(name, f"error: {e}")
    finally:
        if conn:
            log(name, "landing")
            mav_land(conn)


def fly_plane(state):
    name = "fixed-wing"
    info = ASSETS[name]
    cam_info = DRONE_CAMS[name]
    conn = None
    try:
        log(name, "connecting...")
        conn = mav_connect(name, info["udp"])
        wait_for_ekf(name, conn)

        # Plane: TAKEOFF mode → arm → takeoff → switch to GUIDED
        log(name, "TAKEOFF mode")
        mav_set_mode(conn, 13)

        log(name, "arming")
        if not mav_arm(conn, name):
            log(name, "ARM FAILED"); return

        log(name, f"takeoff → {PLANE_ALT}m")
        conn.mav.command_long_send(conn.target_system, conn.target_component,
                                   mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                   0, 0, 0, 0, 0, 0, 0, PLANE_ALT)
        if mav_takeoff(conn, PLANE_ALT, state, timeout=80):
            log(name, "airborne")
        else:
            log(name, "takeoff slow, continuing")

        log(name, "GUIDED mode")
        mav_set_mode(conn, info["guided_mode"])

        threading.Thread(target=mav_heartbeat, args=(conn, state), daemon=True).start()
        threading.Thread(target=run_scanner, args=(name, cam_info["cam"], cam_info, state), daemon=True).start()

        wps = make_circle(*SITE_CENTER, PLANE_RADIUS, N_WAYPOINTS)
        idx = 0
        log(name, f"patrol — {len(wps)} waypoints, {PLANE_RADIUS}m radius")

        orbit_idx = 0
        orbit_wps = []
        last_orbit_center = (0, 0)
        last_goto_time = 0

        while not state.shutdown:
            t = mav_read(conn)
            if t:
                with state.lock:
                    state.positions[name] = t[:3]
                    state.attitudes[name] = t[3:]

            if state.boat_found:
                with state.lock:
                    blat, blon = state.boat_lat, state.boat_lon

                if dist_m(blat, blon, *last_orbit_center) > 50:
                    orbit_wps = make_circle(blat, blon, ORBIT_RADIUS, ORBIT_POINTS)
                    last_orbit_center = (blat, blon)
                    orbit_idx = 0

                if orbit_wps:
                    owlat, owlon = orbit_wps[orbit_idx]
                    if time.time() - last_goto_time > GOTO_INTERVAL:
                        mav_goto(conn, owlat, owlon, PLANE_ALT)
                        last_goto_time = time.time()
                    if t and dist_m(t[0], t[1], owlat, owlon) < ARRIVAL_M:
                        orbit_idx = (orbit_idx + 1) % len(orbit_wps)
                        last_goto_time = 0
                time.sleep(1)
                continue

            wlat, wlon = wps[idx]
            mav_goto(conn, wlat, wlon, PLANE_ALT)

            for _ in range(30):
                if state.shutdown or state.boat_found:
                    break
                t = mav_read(conn)
                if t:
                    with state.lock:
                        state.positions[name] = t[:3]
                        state.attitudes[name] = t[3:]
                    if dist_m(t[0], t[1], wlat, wlon) < ARRIVAL_M:
                        break
                time.sleep(1)

            idx = (idx + 1) % len(wps)

    except Exception as e:
        log(name, f"error: {e}")
    finally:
        if conn:
            log(name, "landing")
            mav_land(conn)

# ---------------------------------------------------------------------------
# Tower scanner threads (YOLO on tower cameras, no flight control)
# ---------------------------------------------------------------------------

def run_tower(name, state):
    info = TOWER_CAMS[name]
    cam_info = info  # same keys: width, height, hfov; mount_pitch = 0 (tower points itself)
    try:
        log(name, "connecting...")
        conn = mav_connect(name, info["udp"])
        threading.Thread(target=mav_heartbeat, args=(conn, state), daemon=True).start()

        # Seed the tower's position immediately from GPS
        t = mav_read(conn)
        if t:
            with state.lock:
                state.positions[name] = t[:3]
                state.attitudes[name] = t[3:]

        # Launch YOLO scanner thread — same as drones
        threading.Thread(target=run_scanner, args=(name, info["cam"], cam_info, state),
                         daemon=True).start()

        # Keep reading telemetry so scanner has fresh attitude
        while not state.shutdown:
            t = mav_read(conn)
            if t:
                with state.lock:
                    state.positions[name] = t[:3]
                    state.attitudes[name] = t[3:]
            time.sleep(1)

    except Exception as e:
        log(name, f"error: {e}")

# ---------------------------------------------------------------------------
# Status printer
# ---------------------------------------------------------------------------

def status_loop(state):
    while not state.shutdown:
        time.sleep(10)
        with state.lock:
            pos = dict(state.positions)
        lines = []
        for n in ["quadcopter", "fixed-wing", "tower-1", "tower-2"]:
            p = pos.get(n)
            if p:
                lines.append(f"  {n:12s}  ({p[0]:.5f}, {p[1]:.5f})  alt={p[2]:5.1f}m")
            else:
                lines.append(f"  {n:12s}  no data")

        flags = []
        if state.boat_found:
            flags.append(f"BOAT({state.boat_lat:.5f},{state.boat_lon:.5f})")
        with _log_lock:
            print(f"\n[{time.strftime('%H:%M:%S')}] --- STATUS ---")
            for l in lines:
                print(l)
            if flags:
                print(f"  {', '.join(flags)}")
            print()

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    print("=" * 60)
    print("ARCTIC SIM-8 — COORDINATED MISSION")
    print("=" * 60)
    print()
    print("Ctrl+C to land drones and exit\n")

    get_detector()  # preload model

    state = State()
    threads = [
        threading.Thread(target=fly_copter, args=(state,), daemon=True),
        threading.Thread(target=fly_plane, args=(state,), daemon=True),
        threading.Thread(target=run_tower, args=("tower-1", state), daemon=True),
        threading.Thread(target=run_tower, args=("tower-2", state), daemon=True),
        threading.Thread(target=status_loop, args=(state,), daemon=True),
    ]
    for t in threads:
        t.start()

    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n\nLanding drones...")
        state.shutdown = True
        for t in threads:
            t.join(timeout=10)
        print("Done.")


if __name__ == "__main__":
    main()
