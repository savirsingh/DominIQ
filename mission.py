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
from tracker import estimate_boat_position, haversine, offset_lat_lon
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

# Camera specs from the SIM models
CAMERAS = {
    "quadcopter": {"width": 960, "height": 720, "hfov": 2.0, "mount_pitch": 0.0},
    "fixed-wing": {"width": 1280, "height": 720, "hfov": 1.204, "mount_pitch": 0.14},
}

# ---------------------------------------------------------------------------
# Camera-based boat position estimation
# ---------------------------------------------------------------------------

def bbox_to_boat_coords(bbox, drone_lat, drone_lon, drone_alt, heading_deg, pitch_deg, cam_name):
    """Estimate boat GPS from a YOLO bounding box + drone telemetry.

    bbox: (x, y, w, h) in pixels — the detection box
    Returns (lat, lon) or None if geometry doesn't work.
    """
    cam = CAMERAS.get(cam_name)
    if not cam or drone_alt < 2:
        return None

    bx, by, bw, bh = bbox
    cx = bx + bw / 2
    cy = by + bh / 2

    img_w, img_h = cam["width"], cam["height"]
    focal_px = (img_w / 2) / math.tan(cam["hfov"] / 2)

    az_offset = math.atan2(cx - img_w / 2, focal_px)
    el_offset = math.atan2(img_h / 2 - cy, focal_px)

    body_pitch_rad = math.radians(pitch_deg)
    cam_pitch = body_pitch_rad - cam["mount_pitch"]
    look_down = -(cam_pitch + el_offset)

    if look_down <= 0.01:
        return None

    ground_dist = drone_alt / math.tan(look_down)
    if ground_dist > 5000 or ground_dist < 0:
        return None

    bearing_rad = math.radians(heading_deg) + az_offset

    boat_lat, boat_lon = offset_lat_lon(drone_lat, drone_lon, ground_dist, bearing_rad)
    return boat_lat, boat_lon

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
        self.locked = False
        self.shutdown = False
        self.asset_positions = {}
        self.asset_attitudes = {}  # {name: (heading_deg, pitch_deg)}

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

    def detect_and_locate(self, img, drone_lat, drone_lon, drone_alt, heading_deg, pitch_deg, cam_name):
        """Detect boat AND estimate its GPS coords from the camera geometry.
        Returns (confidence, boat_lat, boat_lon) or None."""
        dets = self.detect(img)
        if not dets:
            return None
        best_conf, best_bbox = dets[0]
        coords = bbox_to_boat_coords(best_bbox, drone_lat, drone_lon, drone_alt, heading_deg, pitch_deg, cam_name)
        if coords is None:
            return None
        return (best_conf, coords[0], coords[1])

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


def get_telemetry(conn):
    """Get position + attitude. Returns (lat, lon, alt, heading_deg, pitch_deg) or None."""
    gps = conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
    if not gps:
        return None
    att = conn.recv_match(type="ATTITUDE", blocking=True, timeout=1)
    lat = gps.lat / 1e7
    lon = gps.lon / 1e7
    alt = gps.relative_alt / 1e3
    heading = gps.hdg / 100.0
    pitch = math.degrees(att.pitch) if att else 0.0
    if att:
        heading = math.degrees(att.yaw) % 360
    return (lat, lon, alt, heading, pitch)


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
                log(name, "boat found — flying to boat position")
                while state.boat_found and not state.shutdown:
                    with state.lock:
                        blat, blon = state.boat_lat, state.boat_lon
                    if blat != 0.0 and blon != 0.0:
                        send_goto(conn, blat, blon, QUAD_PATROL_ALT)
                    telem = get_telemetry(conn)
                    if telem:
                        with state.lock:
                            state.asset_positions[name] = telem[:3]
                            state.asset_attitudes[name] = telem[3:]
                    time.sleep(1.5)
                continue

            # AI detection while patrolling
            wlat, wlon = waypoints[wp_idx]
            send_goto(conn, wlat, wlon, QUAD_PATROL_ALT)

            while not state.shutdown and not state.boat_found:
                telem = get_telemetry(conn)
                if not telem:
                    time.sleep(0.5)
                    continue
                lat, lon, alt, hdg, pitch = telem
                with state.lock:
                    state.asset_positions[name] = (lat, lon, alt)
                    state.asset_attitudes[name] = (hdg, pitch)

                if time.time() - last_detect > DETECTION_INTERVAL:
                    frame = cam.latest()
                    if frame is not None and detector.model is not None:
                        result = detector.detect_and_locate(frame, lat, lon, alt, hdg, pitch, name)
                        if result:
                            conf, blat, blon = result
                            log(name, f"BOAT DETECTED! conf={conf:.2f} → ({blat:.6f}, {blon:.6f})")
                            with state.lock:
                                state.boat_found = True
                                state.boat_lat = blat
                                state.boat_lon = blon
                            break
                        last_detect = time.time()

                d = dist_between(lat, lon, wlat, wlon)
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
                log(name, "BOAT FOUND — locking on, tracking from camera")
                state.locked = True
                while state.boat_found and not state.shutdown:
                    with state.lock:
                        blat, blon = state.boat_lat, state.boat_lon
                    if blat != 0.0 and blon != 0.0:
                        send_goto(conn, blat, blon, PLANE_PATROL_ALT)
                    telem = get_telemetry(conn)
                    if telem:
                        lat, lon, alt, hdg, pitch = telem
                        with state.lock:
                            state.asset_positions[name] = (lat, lon, alt)
                            state.asset_attitudes[name] = (hdg, pitch)
                        if time.time() - last_detect > DETECTION_INTERVAL:
                            frame = cam.latest()
                            if frame is not None and detector.model is not None:
                                result = detector.detect_and_locate(frame, lat, lon, alt, hdg, pitch, name)
                                if result:
                                    conf, new_blat, new_blon = result
                                    with state.lock:
                                        state.boat_lat = new_blat
                                        state.boat_lon = new_blon
                                last_detect = time.time()
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
                        telem = get_telemetry(conn)
                        if not telem:
                            time.sleep(0.5)
                            continue
                        with state.lock:
                            state.asset_positions[name] = telem[:3]
                            state.asset_attitudes[name] = telem[3:]

                        # Check camera while searching
                        if time.time() - last_detect > DETECTION_INTERVAL:
                            frame = cam.latest()
                            if frame is not None and detector.model is not None:
                                result = detector.detect_and_locate(frame, *telem, name)
                                if result:
                                    conf, blat, blon = result
                                    log(name, f"BOAT DETECTED during search! conf={conf:.2f} → ({blat:.6f}, {blon:.6f})")
                                    with state.lock:
                                        state.boat_found = True
                                        state.boat_lat = blat
                                        state.boat_lon = blon
                                    break
                                last_detect = time.time()

                        d = dist_between(telem[0], telem[1], slat, slon)
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
                telem = get_telemetry(conn)
                if not telem:
                    time.sleep(0.5)
                    continue
                lat, lon, alt, hdg, pitch = telem
                with state.lock:
                    state.asset_positions[name] = (lat, lon, alt)
                    state.asset_attitudes[name] = (hdg, pitch)

                if time.time() - last_detect > DETECTION_INTERVAL:
                    frame = cam.latest()
                    if frame is not None and detector.model is not None:
                        result = detector.detect_and_locate(frame, lat, lon, alt, hdg, pitch, name)
                        if result:
                            conf, blat, blon = result
                            log(name, f"BOAT DETECTED on patrol! conf={conf:.2f} → ({blat:.6f}, {blon:.6f})")
                            with state.lock:
                                state.boat_found = True
                                state.boat_lat = blat
                                state.boat_lon = blon
                            break
                        last_detect = time.time()

                d = dist_between(lat, lon, wlat, wlon)
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


def run_tracker(state, detector):
    """Continuously re-estimate boat position from the fixed-wing's camera."""
    try:
        conn = connect("fixed-wing")
        threading.Thread(target=heartbeat_loop, args=(conn, state), daemon=True).start()
        cam = CameraStream(CAMERA_URLS["fixed-wing"])
        cam_name = "fixed-wing"

        while not state.shutdown:
            if not state.boat_found:
                time.sleep(1)
                continue

            telem = get_telemetry(conn)
            if not telem or telem[2] < 2:
                time.sleep(1)
                continue

            frame = cam.latest()
            if frame is not None and detector.model is not None:
                result = detector.detect_and_locate(frame, *telem, cam_name)
                if result:
                    _, blat, blon = result
                    with state.lock:
                        state.boat_lat = blat
                        state.boat_lon = blon
                        state.locked = True
            time.sleep(DETECTION_INTERVAL)

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
            flags.append("tower-1:MOTION")
        if state.tower2_detected:
            flags.append("tower-2:MOTION")
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
        threading.Thread(target=run_quadcopter, args=(state, detector), name="quadcopter"),
        threading.Thread(target=run_fixed_wing, args=(state, detector), name="fixed-wing"),
        threading.Thread(target=run_tower, args=("tower-1", state), name="tower-1"),
        threading.Thread(target=run_tower, args=("tower-2", state), name="tower-2"),
        threading.Thread(target=run_tracker, args=(state, detector), name="tracker"),
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
