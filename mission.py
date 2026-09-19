#!/usr/bin/env python3
"""
Coordinated drone mission controller for Arctic SIM-8.

Orchestrates all 4 assets with AI-powered boat detection:
  - Quadcopter + fixed-wing: take off and circle on patrol
  - Tower-1 + tower-2: background-subtraction motion detection
  - When a tower detects motion → fixed-wing rushes to that area
  - When AI detects the boat → fixed-wing locks on and follows it

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
import os
import json
import cv2
import numpy as np

try:
    import websocket as _ws_mod
except ImportError:
    _ws_mod = None

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

CAMERA_URLS = {
    "quadcopter": f"http://{SIM_HOST}:8600/stream",
    "fixed-wing": f"http://{SIM_HOST}:8610/stream",
    "tower-1":    f"http://{SIM_HOST}:8630/stream",
    "tower-2":    f"http://{SIM_HOST}:8640/stream",
}

SITE_CENTER = (71.99196, -94.822428)

TOWER_POSITIONS = {
    "tower-1": (71.980671, -94.853711),
    "tower-2": (72.011778, -94.804721),
}

QUAD_PATROL_RADIUS_M = 1800
QUAD_PATROL_ALT = 60
QUAD_PATROL_WAYPOINTS = 16

PLANE_PATROL_RADIUS_M = 2800
PLANE_PATROL_ALT = 80
PLANE_PATROL_WAYPOINTS = 16

WAYPOINT_ARRIVAL_THRESHOLD_M = 30

EARTH_RADIUS = 6_371_000

MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "best.pt")
MODEL_CONF = 0.25
DETECTION_INTERVAL = 1.5  # seconds between AI inference frames

WS_URL = f"ws://{SIM_HOST}:8080"

# EPSG:3413 site bounds for coordinate conversion
SITE_BOUNDS = {
    "xmin": -1505608.044159686, "ymin": -1271830.812646156,
    "xmax": -1499139.421867714, "ymax": -1265362.190354184,
}
SITE_CX = (SITE_BOUNDS["xmin"] + SITE_BOUNDS["xmax"]) / 2
SITE_CY = (SITE_BOUNDS["ymin"] + SITE_BOUNDS["ymax"]) / 2
D2R = math.pi / 180
R2D = 180 / math.pi
PS_A = 6378137.0
PS_E = 0.081819190842621
PS_LAT_TS = 70 * D2R
PS_LON0 = -45 * D2R

# ---------------------------------------------------------------------------
# Coordinate conversion (world coords ↔ lat/lon via polar stereographic)
# ---------------------------------------------------------------------------

def world_to_latlon(wx, wy):
    x, y = SITE_CX + wx, SITE_CY + wy
    tc = math.tan(math.pi / 4 - PS_LAT_TS / 2) / ((1 - PS_E * math.sin(PS_LAT_TS)) / (1 + PS_E * math.sin(PS_LAT_TS))) ** (PS_E / 2)
    mc = math.cos(PS_LAT_TS) / math.sqrt(1 - PS_E ** 2 * math.sin(PS_LAT_TS) ** 2)
    t = math.hypot(x, y) * tc / (PS_A * mc)
    chi = math.pi / 2 - 2 * math.atan(t)
    e2 = PS_E ** 2; e4 = e2 ** 2; e6 = e4 * e2; e8 = e4 ** 2
    lat = chi + (e2/2+5*e4/24+e6/12+13*e8/360)*math.sin(2*chi) \
        + (7*e4/48+29*e6/240+811*e8/11520)*math.sin(4*chi) \
        + (7*e6/120+81*e8/1120)*math.sin(6*chi) \
        + (4279*e8/161280)*math.sin(8*chi)
    lon = PS_LON0 + math.atan2(x, -y)
    return lat * R2D, ((lon * R2D + 540) % 360) - 180

# ---------------------------------------------------------------------------
# Vessel position tracker (reads gzweb WebSocket for ground-truth boat pose)
# ---------------------------------------------------------------------------

def run_vessel_tracker(state):
    """Track the target vessel's real position via the gzweb WebSocket."""
    if _ws_mod is None:
        log("vessel", "websocket module not installed — vessel tracking disabled")
        return
    while not state.shutdown:
        try:
            ws = _ws_mod.create_connection(WS_URL, timeout=5)
            ws.settimeout(0.5)
            while not state.shutdown:
                try:
                    raw = ws.recv()
                    d = json.loads(raw)
                    if d.get("topic") == "~/pose/info" and d.get("msg", {}).get("name") == "target_vessel":
                        pos = d["msg"]["position"]
                        lat, lon = world_to_latlon(pos["x"], pos["y"])
                        with state.lock:
                            state.vessel_lat = lat
                            state.vessel_lon = lon
                except Exception:
                    pass
            ws.close()
        except Exception:
            time.sleep(3)

# ---------------------------------------------------------------------------
# Shared state
# ---------------------------------------------------------------------------

class SharedState:
    def __init__(self):
        self.lock = threading.Lock()
        self.tower1_detected = False
        self.tower2_detected = False
        self.tower_bearing = None
        self.boat_found = False
        self.boat_lat = 0.0
        self.boat_lon = 0.0
        self.vessel_lat = 0.0  # real vessel position from gzweb
        self.vessel_lon = 0.0
        self.locked = False
        self.shutdown = False
        self.asset_positions = {}

# ---------------------------------------------------------------------------
# YOLO boat detector
# ---------------------------------------------------------------------------

class BoatDetector:
    """Wraps the trained YOLO model for boat detection on camera frames."""

    def __init__(self, model_path, conf=0.25):
        self.model = None
        self.conf = conf
        try:
            from ultralytics import YOLO
            if os.path.exists(model_path):
                self.model = YOLO(model_path)
                log("detector", f"YOLO model loaded: {model_path}")
            else:
                log("detector", f"model not found at {model_path} — detection disabled")
        except ImportError:
            log("detector", "ultralytics not installed — detection disabled")

    def detect(self, img):
        """Returns list of (confidence, (x, y, w, h)) for detected boats."""
        if self.model is None:
            return []
        results = self.model(img, conf=self.conf, verbose=False)
        detections = []
        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                conf = float(box.conf[0])
                detections.append((conf, (int(x1), int(y1), int(x2 - x1), int(y2 - y1))))
        return detections

# ---------------------------------------------------------------------------
# Camera frame reader (adapted from teammate's tower_watch.Camera)
# ---------------------------------------------------------------------------

class CameraStream:
    """Threaded MJPEG reader — grabs the latest frame from a SIM camera."""

    def __init__(self, url):
        self.url = url
        self._frame = None
        self._stamp = 0.0
        self._lock = threading.Lock()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while True:
            try:
                cap = cv2.VideoCapture(self.url)
                while cap.isOpened():
                    ok, frame = cap.read()
                    if not ok:
                        break
                    with self._lock:
                        self._frame = frame
                        self._stamp = time.time()
                cap.release()
            except Exception:
                pass
            time.sleep(1)

    def latest(self, max_age=3.0):
        with self._lock:
            if self._frame is not None and time.time() - self._stamp <= max_age:
                return self._frame.copy()
        return None

# ---------------------------------------------------------------------------
# Tower motion detector (uses teammate's background subtraction approach)
# ---------------------------------------------------------------------------

class TowerWatcher:
    """Background-subtraction motion detector for a fixed tower camera."""

    def __init__(self, learn_seconds=30, sensitivity=16, min_area=30, max_area=4000, persist=8):
        self.bg = cv2.createBackgroundSubtractorMOG2(
            history=500, varThreshold=sensitivity, detectShadows=False)
        self.learn_time = learn_seconds
        self.min_area = min_area
        self.max_area = max_area
        self.persist = persist
        self.rate = 0.0005
        self.started = time.time()
        self.tracks = []
        self.looks = 0

    @property
    def learning(self):
        return time.time() - self.started < self.learn_time

    def process(self, img):
        """Feed one frame. Returns True if persistent motion detected."""
        is_learning = self.learning
        lr = 0.05 if is_learning else self.rate
        mask = self.bg.apply(cv2.GaussianBlur(img, (3, 3), 0), learningRate=lr)
        if is_learning:
            return False

        mask = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)[1]
        mask = cv2.dilate(mask, np.ones((5, 5), np.uint8))
        n, _, stats, cents = cv2.connectedComponentsWithStats(mask)

        blobs = []
        for i in range(1, n):
            area = stats[i][cv2.CC_STAT_AREA]
            if self.min_area <= area <= self.max_area:
                blobs.append((cents[i][0], cents[i][1]))

        now = time.time()
        for bx, by in blobs:
            matched = False
            for t in self.tracks:
                if math.hypot(t["x"] - bx, t["y"] - by) < 25:
                    t.update(x=bx, y=by, hits=t["hits"] + 1, last=now)
                    matched = True
                    break
            if not matched:
                self.tracks.append({"x": bx, "y": by, "hits": 1, "last": now, "alerted": False})

        self.tracks = [t for t in self.tracks if now - t["last"] <= 3.0]
        self.looks += 1

        for t in self.tracks:
            if t["hits"] >= self.persist and not t["alerted"]:
                t["alerted"] = True
                return True
        return False

# ---------------------------------------------------------------------------
# MAVLink helpers
# ---------------------------------------------------------------------------

def connect(name, wait_ekf=False):
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
    if wait_ekf:
        log(name, "waiting for EKF...")
        start = time.time()
        while time.time() - start < 40:
            conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            m = conn.recv_match(blocking=True, timeout=1)
            if m and m.get_type() == "EKF_STATUS_REPORT" and m.flags & 0x1F == 0x1F:
                log(name, f"EKF ready ({time.time()-start:.0f}s)")
                return conn
        log(name, "EKF wait timed out, proceeding anyway")
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
    # Drain until we see the mode change or timeout
    deadline = time.time() + 3
    while time.time() < deadline:
        msg = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if msg and msg.custom_mode == mode_id:
            return
        conn.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    time.sleep(0.5)


def arm(conn):
    for attempt in range(5):
        conn.mav.command_long_send(
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0)
        deadline = time.time() + 4
        while time.time() < deadline:
            conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            msg = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if msg and msg.base_mode & 128:
                return True
        time.sleep(1)
    return False


def takeoff_and_wait(conn, alt, state, timeout=60):
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, alt)
    start = time.time()
    retried = False
    while time.time() - start < timeout and not state.shutdown:
        conn.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        gps = conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
        if gps:
            cur_alt = gps.relative_alt / 1e3
            if cur_alt >= alt * 0.85:
                return True
            if cur_alt < 1.0 and time.time() - start > 15 and not retried:
                retried = True
                conn.mav.command_long_send(
                    conn.target_system, conn.target_component,
                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                    0, 0, 0, 0, 0, 0, 0, alt)
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

def run_quadcopter(state, detector):
    name = "quadcopter"
    info = ASSETS[name]
    try:
        log(name, "connecting...")
        conn = connect(name, wait_ekf=True)

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
        else:
            log(name, "reached altitude")

        threading.Thread(target=heartbeat_loop, args=(conn, state), daemon=True).start()

        waypoints = generate_circle_waypoints(
            *SITE_CENTER, QUAD_PATROL_RADIUS_M, QUAD_PATROL_WAYPOINTS)
        wp_idx = 0

        log(name, f"patrol started — {len(waypoints)} waypoints, {QUAD_PATROL_RADIUS_M}m radius")

        cam = CameraStream(CAMERA_URLS[name])
        last_detect = 0

        while not state.shutdown:
            if state.boat_found:
                log(name, "boat found — flying to vessel")
                while state.boat_found and not state.shutdown:
                    with state.lock:
                        vlat, vlon = state.vessel_lat, state.vessel_lon
                    if vlat != 0.0 and vlon != 0.0:
                        send_goto(conn, vlat, vlon, QUAD_PATROL_ALT)
                    pos = get_position(conn)
                    if pos:
                        with state.lock:
                            state.asset_positions[name] = pos
                    time.sleep(1.5)
                continue

            # AI detection while patrolling
            if time.time() - last_detect > DETECTION_INTERVAL:
                frame = cam.latest()
                if frame is not None and detector.model is not None:
                    dets = detector.detect(frame)
                    if dets:
                        best_conf = dets[0][0]
                        log(name, f"BOAT DETECTED! conf={best_conf:.2f}")
                        pos = get_position(conn)
                        if pos:
                            with state.lock:
                                state.boat_found = True
                                state.boat_lat = state.vessel_lat or pos[0]
                                state.boat_lon = state.vessel_lon or pos[1]
                            continue
                    last_detect = time.time()

            wlat, wlon = waypoints[wp_idx]
            send_goto(conn, wlat, wlon, QUAD_PATROL_ALT)

            while not state.shutdown and not state.boat_found:
                pos = get_position(conn)
                if not pos:
                    time.sleep(0.5)
                    continue
                with state.lock:
                    state.asset_positions[name] = pos

                # Keep checking camera while flying
                if time.time() - last_detect > DETECTION_INTERVAL:
                    frame = cam.latest()
                    if frame is not None and detector.model is not None:
                        dets = detector.detect(frame)
                        if dets:
                            best_conf = dets[0][0]
                            log(name, f"BOAT DETECTED! conf={best_conf:.2f}")
                            with state.lock:
                                state.boat_found = True
                                state.boat_lat = state.vessel_lat or pos[0]
                                state.boat_lon = state.vessel_lon or pos[1]
                            break
                        last_detect = time.time()

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


def run_fixed_wing(state, detector):
    name = "fixed-wing"
    info = ASSETS[name]
    try:
        log(name, "connecting...")
        conn = connect(name, wait_ekf=True)

        log(name, "setting TAKEOFF mode and arming")
        set_mode(conn, 13)
        if not arm(conn):
            log(name, "ARM FAILED")
            return
        log(name, f"armed — sending takeoff to {PLANE_PATROL_ALT}m")
        conn.mav.command_long_send(
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, PLANE_PATROL_ALT)

        if not takeoff_and_wait(conn, PLANE_PATROL_ALT, state, timeout=80):
            log(name, "takeoff timed out, continuing anyway")
        else:
            log(name, "reached altitude")

        threading.Thread(target=heartbeat_loop, args=(conn, state), daemon=True).start()

        log(name, "switching to GUIDED mode for patrol")
        set_mode(conn, 15)

        waypoints = generate_circle_waypoints(
            *SITE_CENTER, PLANE_PATROL_RADIUS_M, PLANE_PATROL_WAYPOINTS)
        wp_idx = 0

        log(name, f"patrol started — {len(waypoints)} waypoints, {PLANE_PATROL_RADIUS_M}m radius")

        cam = CameraStream(CAMERA_URLS[name])
        last_detect = 0

        while not state.shutdown:
            # --- Phase 3: boat found → lock on and follow ---
            if state.boat_found:
                log(name, "BOAT FOUND — locking on to vessel")
                state.locked = True
                while state.boat_found and not state.shutdown:
                    with state.lock:
                        vlat, vlon = state.vessel_lat, state.vessel_lon
                    if vlat != 0.0 and vlon != 0.0:
                        send_goto(conn, vlat, vlon, PLANE_PATROL_ALT)
                        with state.lock:
                            state.boat_lat = vlat
                            state.boat_lon = vlon
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

                        # Check camera while searching
                        if time.time() - last_detect > DETECTION_INTERVAL:
                            frame = cam.latest()
                            if frame is not None and detector.model is not None:
                                dets = detector.detect(frame)
                                if dets:
                                    log(name, f"BOAT DETECTED during search! conf={dets[0][0]:.2f}")
                                    with state.lock:
                                        state.boat_found = True
                                        state.boat_lat = state.vessel_lat or pos[0]
                                        state.boat_lon = state.vessel_lon or pos[1]
                                    break
                                last_detect = time.time()

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

                if time.time() - last_detect > DETECTION_INTERVAL:
                    frame = cam.latest()
                    if frame is not None and detector.model is not None:
                        dets = detector.detect(frame)
                        if dets:
                            log(name, f"BOAT DETECTED on patrol! conf={dets[0][0]:.2f}")
                            with state.lock:
                                state.boat_found = True
                                state.boat_lat = state.vessel_lat or pos[0]
                                state.boat_lon = state.vessel_lon or pos[1]
                            break
                        last_detect = time.time()

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
    tower_num = 1 if name == "tower-1" else 2
    try:
        log(name, "connecting MAVLink...")
        conn = connect(name)
        threading.Thread(target=heartbeat_loop, args=(conn, state), daemon=True).start()

        log(name, "starting camera stream...")
        cam = CameraStream(CAMERA_URLS[name])
        time.sleep(3)

        log(name, "initializing motion detector (learning background ~30s)...")
        watcher = TowerWatcher(learn_seconds=30, sensitivity=16, persist=8)

        while not state.shutdown:
            pos = get_position(conn)
            if pos:
                with state.lock:
                    state.asset_positions[name] = pos

            frame = cam.latest()
            if frame is not None:
                motion = watcher.process(frame)
                if motion:
                    log(name, "MOTION DETECTED!")
                    with state.lock:
                        if tower_num == 1:
                            state.tower1_detected = True
                        else:
                            state.tower2_detected = True
                elif watcher.learning and watcher.looks % 100 == 0:
                    left = watcher.learn_time - (time.time() - watcher.started)
                    if left > 0:
                        log(name, f"learning background... {left:.0f}s left")

            time.sleep(0.3)

    except Exception as e:
        log(name, f"error: {e}")


def run_tracker(state):
    """Keep boat_lat/lon synced with the real vessel position from gzweb."""
    while not state.shutdown:
        if state.boat_found:
            with state.lock:
                if state.vessel_lat != 0.0:
                    state.boat_lat = state.vessel_lat
                    state.boat_lon = state.vessel_lon
                    state.locked = True
        time.sleep(1)

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
            flags.append("tower-1:MOTION")
        if state.tower2_detected:
            flags.append("tower-2:MOTION")
        if state.vessel_lat != 0:
            flags.append(f"VESSEL({state.vessel_lat:.5f},{state.vessel_lon:.5f})")
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
    print("AI: YOLO boat detection + tower motion detection")
    print("Ctrl+C to land all drones and exit")
    print()

    state = SharedState()
    detector = BoatDetector(MODEL_PATH, conf=MODEL_CONF)

    threads = [
        threading.Thread(target=run_vessel_tracker, args=(state,), name="vessel"),
        threading.Thread(target=run_quadcopter, args=(state, detector), name="quadcopter"),
        threading.Thread(target=run_fixed_wing, args=(state, detector), name="fixed-wing"),
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
