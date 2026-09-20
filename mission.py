#!/usr/bin/env python3
"""
Coordinated drone mission controller for Arctic SIM-8.

Orchestrates all 4 assets with AI-powered boat detection:
  - Quadcopter: takes off and flies a lawnmower grid over a search corridor, then RTL
  - Fixed-wing: takes off and flies the same search corridor the other way round
  - Tower-1 + tower-2: sweep the range marked with tower_aim.py (left bound, right bound, tilt);
    in each area learn the background for 10 s, watch for motion, then turn to the next area
    (background-subtraction motion detection). A tower with no marked range sweeps the horizon.
  - When a tower detects motion → fixed-wing rushes to that area
  - When AI detects the boat → the asset that saw it tracks it: the quadcopter keeps it centred in its
    camera, slows down as it closes in, estimates the boat's velocity from the camera fixes and then
    orbits it, matching its speed; the fixed-wing flies to (and loiters around) the filtered position

Usage:
    python3 mission.py              # the shared sim over WireGuard (default)
    python3 mission.py --sim local  # your own docker compose sim (or SIM_TARGET=local)
    python3 mission.py --tower-config ../htn-2026/anomaly-demo/tower_aim.json
                                    # where the towers' marked sweep ranges are read from

Press Ctrl+C to land all drones and shut down.
"""

from pymavlink import mavutil
from tracker import estimate_from_pixel
from tower_watch import AIM_FILE, Camera, Tower, Watcher, describe, wrap180
import argparse
import json
import sim_config
import threading
import math
import time
from types import SimpleNamespace
import sys
import os
import traceback
import json
import cv2

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

# Host, MAVLink ports and camera URLs come from sim_config (--sim local|remote, default remote).
GUIDED_MODES = {"quadcopter": 4, "fixed-wing": 15}

# Both sims are the fort_ross site, so the centre is the same.
SITE_CENTER = (71.99196, -94.822428)

# Tower positions of the shared (remote) sim. The local sim places them from its own .env, so in
# local mode main() replaces these with what each tower reports over MAVLink.
TOWER_POSITIONS = {
    "tower-1": (71.980671, -94.853711),
    "tower-2": (72.011778, -94.804721),
}

# Quadcopter search: lanes parallel to the line between these two towers, covering SEARCH_WIDTH_M
# centred on it, joined into one back-and-forth path (lifted from anomaly-demo/patrol.py).
SEARCH_FROM_TOWER = "tower-1"
SEARCH_TO_TOWER = "tower-2"
SEARCH_WIDTH_M = 1000
SEARCH_LANE_SPACING_M = 150
QUAD_PATROL_ALT = 60       # metres above the takeoff point, not above the water
QUAD_SPEED_MS = 8          # ground speed while patrolling
QUAD_REACH_M = 8           # waypoint counts as reached within this many metres

# Following the boat: keep it in the centre of the quadcopter's fixed camera. Sideways error turns the
# nose (boat on the left -> yaw left); vertical error moves the copter forward/back, since the camera
# cannot tilt and range is the only thing that moves the boat up or down in the frame.
FOLLOW_YAW_KP = 1.0            # rad/s of yaw rate per rad of horizontal error
FOLLOW_YAW_MAX_DEG_S = 25      # cap on the turn rate, so the nose never whips round
FOLLOW_YAW_DEADBAND_DEG = 2.0  # no turning while the boat is within this of the centre
FOLLOW_FWD_KP = 40.0           # m/s per rad of vertical error (1 deg of error is ~20 m of range here)
FOLLOW_FWD_MAX_MS = QUAD_SPEED_MS
FOLLOW_BACK_MAX_MS = 3.0       # backing off is slower than closing in
FOLLOW_TILT_DEADBAND_DEG = 1.5
FOLLOW_STALE_S = 1.5           # no fresh sighting for this long -> stop trusting where it sat in the frame
FOLLOW_SEARCH_MS = 3.0         # ...and instead swing toward the bearing it was last seen on and creep in
FOLLOW_SEARCH_MAX_YAW_DEG = 20 # only creep forward once the nose is within this of that bearing
# A boat narrower than this share of the frame is too small to detect reliably: close in on it even
# if that means it sits below the centre of the frame, by up to FOLLOW_CLOSE_TILT_MAX_DEG (for a
# boat only a few pixels wide; the allowance fades out as it grows to FOLLOW_MIN_BOX_FRAC).
FOLLOW_MIN_BOX_FRAC = 0.05
FOLLOW_CLOSE_TILT_MAX_DEG = 15.0
# Easing off as it gets close, so it does not fly over the boat (the fixed camera loses it out of the
# bottom of the frame): once the boat is wider than FOLLOW_SLOW_START_FRAC of the frame, the closing
# speed limit falls from FOLLOW_FWD_MAX_MS to FOLLOW_NEAR_MAX_MS by FOLLOW_SLOW_END_FRAC. That leaves
# the nose time to swing round and the follow to start. A boat pulling away shrinks and gets its speed back.
FOLLOW_SLOW_START_FRAC = 0.05
FOLLOW_SLOW_END_FRAC = 0.12
FOLLOW_NEAR_MAX_MS = 1.5

# --- Estimating the boat's velocity --------------------------------------------------------------
# Every detection is projected onto the water (locate_boat) and fed to an alpha-beta filter that
# keeps the boat's position and velocity. A fix only counts if it was taken in a good pose: close
# enough that the projection is sound, and not while turning or banking, when a small timing error
# in the attitude becomes tens of metres.
CAMERA_LATENCY_S = 0.15          # capture -> our read of the frame; the pose is looked up this far back (a guess)
FIX_MAX_RANGE_M = 700            # beyond this one pixel or degree is 10s of metres
FIX_MAX_YAWRATE_DEG_S = 3.0
FIX_MAX_ROLL_DEG = {"quadcopter": 10.0, "fixed-wing": 8.0}
FIX_WEIGHT = {"quadcopter": 1.0, "fixed-wing": 0.5}    # the plane's shallow, banking view is trusted less
TRACK_ALPHA = 0.2                # steady-state pull of a fix on the position estimate
TRACK_BETA = 0.01                # ... and on the velocity estimate (per fix, scaled by the time between them);
                                 # measured: ~0.3 m/s speed error at 10 m of fix noise, ~13 s to settle
TRACK_MIN_DT_S = 0.3             # fixes closer together than this are skipped (velocity gain 1/dt)
TRACK_GATE_M = 50                # a fix further than this from the prediction is rejected as an outlier
TRACK_REINIT_REJECTS = 6         # ...unless this many in a row are, then the track starts over from the new fix
BOAT_SPEED_MAX_MS = 10.0
TRACK_MIN_FIXES = 8              # the speed counts once this many fixes...
TRACK_MIN_SPAN_S = 8.0           # ...span this long...
TRACK_STABLE_WINDOW_S = 4.0
TRACK_STABLE_MS = 1.0            # ...and the speed of the last few seconds stayed within this band
TRACK_UNSTABLE_MS = 2.5          # once locked, only a band wider than this unlocks it (fix noise alone can widen it)
TRACK_MAX_AGE_S = 3.0            # to START an orbit the last fix must be this recent
TRACK_COAST_S = 20.0             # the velocity keeps being used, and the position extrapolated, this long without fixes

# --- Orbiting it ----------------------------------------------------------------------------------
# Circle the boat with the nose kept on it (the camera is fixed): sideways velocity plus a yaw rate
# of speed/radius, on top of the boat's own velocity so the circle travels with it. The radius is
# whatever range it had when the orbit began, held by keeping the boat on the same row of the frame.
ORBIT_SPEED_MS = 3.0
ORBIT_DIRECTION = 1              # +1 clockwise seen from above, -1 anticlockwise
ORBIT_ALIGN_DEG = 5.0            # start only with the boat this close to the frame centre sideways...
ORBIT_SETTLE_MS = 1.0            # ...and the range loop asking for less than this closing speed...
ORBIT_SETTLE_S = 3.0             # ...for this long
ORBIT_R_MIN_M = 50
ORBIT_R_MAX_M = 450
ORBIT_MAX_TILT_DRIFT_DEG = 12.0  # give up the orbit if the boat wanders this far from its row
BOAT_LOST_FRAMES_PLANE = 150     # the plane's forward camera sees the boat only briefly each lap of its orbit
JOIN_STANDOFF_M = 100            # a copter joining a track stops this far short of the boat's estimate, boat in view
JOIN_HANDOFF_M = 300             # ...and without a fresh sighting of its own it keeps flying at the estimate until this close

PLANE_PATROL_ALT = 80
STALL_WARN_AIRSPEED_MS = 10.0   # the plane cruises at ~12.4 m/s and stalled below ~8: warn well before that
STALL_WARN_SINK_MS = -2.5
PLANE_LOITER_RADIUS_M = 120     # WP_LOITER_RAD in the sim's plane.parm: the orbit around a goto target
PLANE_TOWER_ORBIT_RADIUS_M = 400  # orbit around a tower that saw motion (a wide circle, so a shallow, steady bank)
PLANE_TAKEOFF_ALT = 60      # TAKEOFF mode climbs to TKOFF_ALT (plane.parm) and holds there, whatever NAV_TAKEOFF asks

# A waypoint counts as reached within this many metres. In GUIDED mode the plane orbits a goto target
# at WP_LOITER_RAD (120 m in the sim's plane.parm), so it never gets closer than that: the arrival
# radius has to be larger, or it circles the first waypoint forever.
WAYPOINT_ARRIVAL_THRESHOLD_M = 150

# Tower sweep (logic from anomaly-demo/tower_aim.py): in each area the tower holds still, learns the
# background, watches for movement, then turns to the next area. The areas are spread evenly across
# the range marked in tower_aim.py ([ left bound, ] right bound, t tilt; saved in tower_aim.json), at
# the marked tilt, and the tower reverses at each bound. A tower with no marked range falls back to
# stepping TOWER_STEP_DEG at a time along the whole horizon, reversing at the end of its pan travel.
TOWER_LEARN_S = 10         # holding still to learn the background
TOWER_WATCH_S = 20         # then watching, before moving on
TOWER_STEP_DEG = 50        # the camera's field of view is 60 deg, so neighbouring areas overlap a little
TOWER_OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "tower_output")

EARTH_RADIUS = 6_371_000
EARTH_M_PER_DEG = 111_320.0  # metres per degree of latitude

COPTER_RTL_MODE = 6
# SET_POSITION_TARGET_GLOBAL_INT: use position + yaw, ignore velocity/accel/yaw-rate.
TYPE_MASK_POS_YAW = 8 | 16 | 32 | 64 | 128 | 256 | 2048
# SET_POSITION_TARGET_LOCAL_INT: use velocity + yaw rate, ignore position/accel/yaw.
TYPE_MASK_VEL_YAWRATE = 1 | 2 | 4 | 64 | 128 | 256 | 1024

MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "best.pt")
MODEL_CONF = 0.25
MODEL_CONF_TRACK = 0.20   # once a boat is being tracked, accept slightly fainter detections of it than it took to find it (0.15 let too much through)
DETECTION_INTERVAL = 1.5  # seconds between AI inference frames while searching
TRACK_INTERVAL = 0.5      # seconds between inference passes on the asset that is tracking the boat
TOWER_FIX_TIMEOUT_S = 40   # how long to wait for a tower to report a real position at startup
# --- Confirming a boat before anything else is redirected ------------------------------------------
# The first detection is only a candidate. The asset that saw it engages it (the quad centres it, the plane
# heads for it) so it stays in view, and it counts as a boat once enough detections that can be placed on the
# water agree on where it is. The window is generous because the boat flickers in and out of detection.
CONFIRM_HITS = 4                 # detections that agree...
CONFIRM_WINDOW_FRAMES = 40       # ...within this many new camera frames (~20-40 s), not necessarily consecutive
CONFIRM_MAX_S = 45.0             # wall-clock cap, in case frames stop arriving
CONFIRM_GATE_M = 250             # a hit must lie this close to the earlier hits' mean position
CONFIRM_MAX_TURN_DEG = 100       # the plane will not turn more than this to engage a candidate (a 122 deg turn stalled it)
REJECT_MEMORY_S = 180.0          # a candidate that failed is ignored...
REJECT_RADIUS_M = 300.0          # ...within this distance of where it was, for this long
# Once tracking, a detection only counts as the boat if it is this close to where the track says it is
# (plus a growing allowance for each second since it was last seen: it moves). Anything else is a miss.
TRACK_HIT_GATE_M = {"quadcopter": 300, "fixed-wing": 500}
TRACK_HIT_GATE_GROW_MS = 5.0
TRACK_HIT_GATE_GROW_MAX_S = 60.0

# --- Trusting the towers -----------------------------------------------------------------------------
# A tower call is a timestamped event (state.tower_calls / tower_seq / last_tower) and the newest call wins.
# The plane swings to the tower that called last, and a new call breaks off a candidate that is still only
# a single sighting (one with more hits keeps confirming, or noisy towers would make confirming impossible).
TOWER_RETARGET_MIN_S = 20.0      # the plane will not swing between towers more often than this (a 180 deg turn at 12 m/s risks a stall)
CONFIRM_TOWER_ABORT_HITS = 2     # a tower call abandons a candidate with fewer hits than this
QUAD_RESPONDS_TO_TOWERS = True   # the quad goes to a tower that calls and searches around it...
QUAD_TOWER_RADIUS_M = 250        # ...on a circle this far out...
QUAD_TOWER_ACTIVE_S = 120.0      # ...until the tower has been quiet this long, then it resumes its lane search

# --- A new lock has to prove itself ------------------------------------------------------------------
# Confirmation is generous (the boat flickers), so right after it the lock is fragile on purpose: it must
# collect PROBATION_HITS hits, from any camera, within PROBATION_S of being confirmed, or it is forgotten.
PROBATION_HITS = 10
PROBATION_S = 45.0
BOAT_LOST_FRAMES = 25     # consecutive new frames with no detection before the boat is dropped (~25 s)

# Camera specs read from the sim source (arctic-sim/sim/models). Both are fixed, forward-looking
# mounts tilted down about the airframe's right axis; neither rolls about its optical axis.
#   quadcopter: gimbal_small_2d webcam, HFOV 2.0 rad, 960x720, iris mount roll 1.9199 -> 20.00 deg down
#   fixed-wing: skywalker_x8 fpv camera, HFOV 1.204 rad, 1280x720, pose pitch 0.140 rad -> 8.02 deg down
CAMERAS = {
    "quadcopter": {"hfov": 2.0,   "tilt_down_deg": 20.0},
    "fixed-wing": {"hfov": 1.204, "tilt_down_deg": 8.02},
}

# The water is a flat plane at sea level (world z = 0), and the sim keeps ArduPilot's AMSL altitude
# equal to world z. relative_alt is height above the spawn point (75.79 m AMSL at fort_ross), so
# it is NOT height above water; GLOBAL_POSITION_INT.alt (AMSL) is.
WATER_AMSL_M = 0.0

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
        self.boat_lat = 0.0       # newest raw fix (or a seed); the filtered position is in boat_track
        self.boat_lon = 0.0
        self.boat_track = BoatTrack()
        self.found_by = None      # the asset that saw it first (for the log): every asset tracks it once found
        self.tracker_misses = {}  # asset -> (consecutive frames without the boat, its limit), for each asset tracking
        self.rejected = []        # (lat, lon, until) of candidates that failed confirmation: ignored for a while
        self.tower_seq = 0        # counts tower calls
        self.last_tower = None    # the tower that called last
        self.tower_calls = {}     # tower name -> time of its latest call
        self.locked_at = 0.0      # when the current boat was confirmed (0 = no probation to apply)
        self.lock_hits = 0        # hits on it since, from any camera
        self.lock_established = False
        self.boat_lost = False    # set when a tracked boat was dropped, cleared on the next sighting
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

    def detect(self, img, conf=None):
        """Returns list of (confidence, (x, y, w, h)) for detected boats; `conf` overrides the threshold."""
        if self.model is None:
            return []
        results = self.model(img, conf=self.conf if conf is None else conf, verbose=False)
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

    def latest_stamped(self, max_age=3.0):
        """(frame, stamp) for the newest frame that is not stale, else (None, 0.0)."""
        with self._lock:
            if self._frame is not None and time.time() - self._stamp <= max_age:
                return self._frame.copy(), self._stamp
        return None, 0.0

    def latest(self, max_age=3.0):
        return self.latest_stamped(max_age)[0]

    def newest(self, after, max_age=3.0):
        """(frame, stamp) for a frame newer than `after` and not stale, else (None, after)."""
        with self._lock:
            if self._frame is not None and self._stamp > after and time.time() - self._stamp <= max_age:
                return self._frame.copy(), self._stamp
        return None, after

# ---------------------------------------------------------------------------
# Time-synced telemetry, and the boat's filtered track
# ---------------------------------------------------------------------------

class TelemetryLog:
    """Timestamped ATTITUDE and GLOBAL_POSITION_INT of one vehicle.

    By the time inference has run, the frame is a fraction of a second old, and a vehicle turning at
    25 deg/s changes heading by ~12 deg in half a second: ~80 m of error in the projected boat position
    at 370 m. So a frame is projected with the pose the vehicle had when the frame was taken
    (interpolated between samples), not the latest one. Filled by a pymavlink message hook, so it sees
    every message whichever code reads the connection.
    """

    def __init__(self, conn, keep_s=8.0, rate_hz=20):
        self._att, self._gps = [], []
        self._keep = keep_s
        self._lock = threading.Lock()
        conn.message_hooks.append(self._on_message)
        for msg_id in (30, 33):               # ATTITUDE, GLOBAL_POSITION_INT: more than the 4 Hz stream (best effort)
            conn.mav.command_long_send(
                conn.target_system, conn.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0, msg_id, int(1e6 / rate_hz), 0, 0, 0, 0, 0)

    def _on_message(self, conn, msg):
        kind = msg.get_type()
        if kind == "ATTITUDE":
            buf = self._att
        elif kind == "GLOBAL_POSITION_INT":
            buf = self._gps
        else:
            return
        t = getattr(msg, "_timestamp", None) or time.time()
        with self._lock:
            buf.append((t, msg))
            while buf and t - buf[0][0] > self._keep:
                buf.pop(0)

    @staticmethod
    def _bracket(buf, t):
        """(older msg, newer msg, fraction between them) around time t; clamps at either end."""
        if not buf:
            return None
        if t <= buf[0][0]:
            return buf[0][1], buf[0][1], 0.0
        if t >= buf[-1][0]:
            return buf[-1][1], buf[-1][1], 0.0
        for (t0, m0), (t1, m1) in zip(buf, buf[1:]):
            if t0 <= t <= t1:
                return m0, m1, (t - t0) / (t1 - t0) if t1 > t0 else 0.0
        return buf[-1][1], buf[-1][1], 0.0

    def at(self, t):
        """The pose at wall-clock time t as (lat, lon, alt AMSL m, yaw, pitch, roll rad, yawspeed rad/s), or None."""
        with self._lock:
            att, gps = self._bracket(self._att, t), self._bracket(self._gps, t)
        if att is None or gps is None:
            return None
        (a0, a1, k), (g0, g1, kg) = att, gps
        mix = lambda x0, x1, f: x0 + (x1 - x0) * f
        return SimpleNamespace(
            lat=mix(g0.lat, g1.lat, kg) / 1e7, lon=mix(g0.lon, g1.lon, kg) / 1e7,
            alt=mix(g0.alt, g1.alt, kg) / 1e3,
            yaw=a0.yaw + wrap_pi(a1.yaw - a0.yaw) * k,
            pitch=mix(a0.pitch, a1.pitch, k), roll=mix(a0.roll, a1.roll, k),
            yawspeed=mix(a0.yawspeed, a1.yawspeed, k))


class BoatTrack:
    """The boat's position and velocity, filtered from the noisy per-frame ground fixes.

    An alpha-beta filter in east/north metres about the first fix: predict forward with the velocity,
    then pull position and velocity toward each new fix. A fix far from the prediction is rejected
    (the track starts over if that keeps happening). The velocity is only handed out once it has
    converged; before that the boat is treated as stationary. Thread-safe: the quadcopter feeds it,
    the fixed-wing reads it.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._clear()

    def _clear(self):
        self._origin = None
        self._t = self._t0 = 0.0
        self._e = self._n = self._ve = self._vn = 0.0
        self._fixes = self._rejects = 0
        self._speeds = []                           # (time, speed) of recent accepted updates
        self._locked = False                        # the velocity has converged (latched, see _converged)

    def reset(self):
        with self._lock:
            self._clear()

    def has_fix(self):
        with self._lock:
            return self._origin is not None

    def _local(self, lat, lon):
        lat0, lon0 = self._origin
        return ((lon - lon0) * EARTH_M_PER_DEG * math.cos(math.radians(lat0)),
                (lat - lat0) * EARTH_M_PER_DEG)

    def update(self, lat, lon, t, weight=1.0):
        """Feed one fix taken at wall-clock time t. Returns whether it was used."""
        with self._lock:
            if self._origin is None:
                self._origin, self._t, self._t0, self._fixes = (lat, lon), t, t, 1
                return True
            dt = t - self._t
            if dt < TRACK_MIN_DT_S:
                return False
            if dt > TRACK_COAST_S:
                # Unseen so long that the prediction is worthless (and would make the outlier gate throw
                # the boat away when it reappears): start over from this fix.
                self._clear()
                self._origin, self._t, self._t0, self._fixes = (lat, lon), t, t, 1
                return True
            ze, zn = self._local(lat, lon)
            pe, pn = self._e + self._ve * dt, self._n + self._vn * dt
            re, rn = ze - pe, zn - pn
            if math.hypot(re, rn) > TRACK_GATE_M + 2.0 * min(dt, 30.0):
                self._rejects += 1
                if self._rejects >= TRACK_REINIT_REJECTS:
                    self._clear()
                    self._origin, self._t, self._t0, self._fixes = (lat, lon), t, t, 1
                    return True
                return False
            self._rejects = 0
            # Growing memory: the first fixes pull hard (a fresh track knows nothing), settling to the
            # steady gains, so the velocity is usable in seconds rather than after a slow ramp.
            k = self._fixes + 1
            a = max(TRACK_ALPHA, 2 * (2 * k - 1) / (k * (k + 1))) * weight
            b = max(TRACK_BETA, 6 / (k * (k + 1))) * weight
            self._e, self._n = pe + a * re, pn + a * rn
            self._ve += b / dt * re
            self._vn += b / dt * rn
            speed = math.hypot(self._ve, self._vn)
            if speed > BOAT_SPEED_MAX_MS:
                self._ve, self._vn = (self._ve * BOAT_SPEED_MAX_MS / speed, self._vn * BOAT_SPEED_MAX_MS / speed)
                speed = BOAT_SPEED_MAX_MS
            self._t, self._fixes = t, self._fixes + 1
            self._speeds = [(ts, s) for ts, s in self._speeds if t - ts <= TRACK_STABLE_WINDOW_S] + [(t, speed)]
            return True

    def _converged(self):
        """Enough fixes over enough time, with a steady speed. Latched: it locks at a band of
        TRACK_STABLE_MS and only unlocks above TRACK_UNSTABLE_MS, and with too few recent samples (a gap
        in detections) it stays as it was, so noise or a short blackout do not throw away a good estimate."""
        if self._origin is None or self._fixes < TRACK_MIN_FIXES or self._t - self._t0 < TRACK_MIN_SPAN_S:
            self._locked = False
            return False
        speeds = [s for _, s in self._speeds]
        if len(speeds) >= 3:
            self._locked = max(speeds) - min(speeds) <= (TRACK_UNSTABLE_MS if self._locked else TRACK_STABLE_MS)
        return self._locked

    def converged(self, now):
        """The velocity is trustworthy (and recent enough to keep coasting on)."""
        with self._lock:
            return self._converged() and now - self._t <= TRACK_COAST_S

    def ready(self, now):
        """Converged, and a fix arrived just now: good enough to start an orbit."""
        with self._lock:
            return self._converged() and now - self._t <= TRACK_MAX_AGE_S

    def velocity(self, now):
        """(east, north) m/s once converged and not stale, else (0, 0)."""
        with self._lock:
            if self._converged() and now - self._t <= TRACK_COAST_S:
                return self._ve, self._vn
            return 0.0, 0.0

    def position(self, now):
        """(lat, lon) of the boat at `now`, extrapolated from the last fix, or None with no fix yet."""
        with self._lock:
            if self._origin is None:
                return None
            dt = min(max(now - self._t, 0.0), TRACK_COAST_S)
            ve, vn = (self._ve, self._vn) if self._converged() else (0.0, 0.0)
            return to_latlon(self._origin, self._e + ve * dt, self._n + vn * dt)

    def describe(self, now):
        """One line for the log: the speed and heading, or how far from converged it is."""
        with self._lock:
            if self._origin is None:
                return "no boat track yet"
            if not self._converged():
                return f"speed not converged yet ({self._fixes} fixes over {self._t - self._t0:.0f} s)"
            speed = math.hypot(self._ve, self._vn)
            heading = math.degrees(math.atan2(self._ve, self._vn)) % 360
            return f"boat speed {speed:.1f} m/s heading {heading:.0f} deg ({self._fixes} fixes)"

# ---------------------------------------------------------------------------
# MAVLink helpers
# ---------------------------------------------------------------------------

# Optional feed for the WebXR map (webxr/bridge/feed.py). Off unless --webxr-feed is given.
_WEBXR_FEED = None


def _publish_position(name):
    """pymavlink message hook: each GLOBAL_POSITION_INT this connection receives also goes to the map.

    Hooks run whichever code reads the connection, so the map adds no second MAVLink client.
    """
    kind = sim_config.ASSETS[name]["type"]

    def hook(conn, msg):
        if _WEBXR_FEED is not None and msg.get_type() == "GLOBAL_POSITION_INT":
            _WEBXR_FEED.update(name, kind, msg.lat / 1e7, msg.lon / 1e7, msg.alt / 1000.0)
    return hook


def connect(name, wait_ekf=False):
    conn = mavutil.mavlink_connection(sim_config.mavlink_url(name), source_system=255)
    conn.message_hooks.append(_publish_position(name))
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


def crash_reason(e):
    """'ZeroDivisionError: float division by zero (mission.py:463 in corridor)': type, message, innermost frame."""
    tb = traceback.extract_tb(e.__traceback__)
    where = f" ({os.path.basename(tb[-1].filename)}:{tb[-1].lineno} in {tb[-1].name})" if tb else ""
    return f"{type(e).__name__}: {e}{where}"


def set_mode(conn, mode_id):
    conn.mav.set_mode_send(
        conn.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)
    # Drain until we see the mode change or timeout. Returns whether the autopilot confirmed it.
    deadline = time.time() + 3
    last = None
    while time.time() < deadline:
        msg = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if msg and msg.custom_mode == mode_id:
            return True
        if msg:
            last = msg.custom_mode
        conn.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    time.sleep(0.5)
    set_mode.last_seen = last
    return False


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
    """The vehicle's latest position.

    The autopilot sends GLOBAL_POSITION_INT several times a second but each control loop looks once per
    iteration, and recv_match hands back the OLDEST queued message. Reading one per pass therefore falls
    further behind every second (about 4x slow at 4 Hz with a 1 s loop): the plane looked to be
    orbiting its takeoff point while it was really somewhere else. So take everything already queued,
    keep the newest, and only block when nothing is waiting.
    """
    gps = None
    while True:
        msg = conn.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
        if msg is None:
            break
        gps = msg
    if gps is None:
        gps = conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
    if not gps:
        return None
    return (gps.lat / 1e7, gps.lon / 1e7, gps.relative_alt / 1e3)


def send_goto(conn, lat, lon, alt, yaw=None):
    """Fly to a position. With yaw (radians clockwise from north) the nose is held on it too."""
    conn.mav.set_position_target_global_int_send(
        0, conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000 if yaw is None else TYPE_MASK_POS_YAW,
        int(lat * 1e7), int(lon * 1e7), alt,
        0, 0, 0,
        0, 0, 0,
        yaw or 0, 0)


def plane_goto(name, conn, lat, lon, alt, tries=3, radius=PLANE_LOITER_RADIUS_M):
    """Send the fixed-wing to a location. Returns whether the autopilot accepted it.

    ArduPlane reads only the ALTITUDE from SET_POSITION_TARGET_GLOBAL_INT (its handler in GCS_Mavlink.cpp
    never looks at lat/lon), so send_goto() makes a plane climb and otherwise circle wherever it entered
    GUIDED. MAV_CMD_DO_REPOSITION is the command that sets a plane's guided target; it switches to GUIDED
    itself and is acknowledged, so a refusal shows up here instead of as a plane that silently ignores us.
    Altitude is metres above home (MAV_FRAME_GLOBAL_RELATIVE_ALT). `radius` is the loiter radius around the
    target; ArduPlane only applies it when it is > 0 (0 means "leave it as it was", not "reset"), so it is always
    sent explicitly: otherwise a wide orbit would stick for every later goto.
    """
    result = None
    for _ in range(tries):
        conn.mav.command_int_send(
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_DO_REPOSITION, 0, 0,
            -1,                                                     # ground speed: default
            mavutil.mavlink.MAV_DO_REPOSITION_FLAGS_CHANGE_MODE,    # switch to GUIDED if needed
            radius,                                                 # param3: loiter radius, metres
            float("nan"),                                           # loiter direction: clockwise
            int(lat * 1e7), int(lon * 1e7), alt)
        deadline = time.time() + 2
        while time.time() < deadline:
            msg = conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
            if msg is not None and msg.command == mavutil.mavlink.MAV_CMD_DO_REPOSITION:
                result = msg.result
                break
        if result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            return True
    log(name, "WARNING: the plane did not accept the reposition command "
              f"({'no answer' if result is None else 'result ' + str(result)}); it will keep circling where it is")
    return False


def send_velocity_yawrate(conn, forward_ms, yaw_rate, right_ms=0.0):
    """Body-frame forward and right speed (m/s, negative = backwards / left) and yaw rate (rad/s,
    + = clockwise). Altitude is held (vz = 0); ArduCopter also zeroes the velocity if these stop arriving."""
    conn.mav.set_position_target_local_ned_send(
        0, conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        TYPE_MASK_VEL_YAWRATE,
        0, 0, 0,
        forward_ms, right_ms, 0,
        0, 0, 0,
        0, yaw_rate)


def set_speed(conn, mps):
    """Ground speed in m/s; a command, not a parameter write."""
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
        0, 1, mps, -1, 0, 0, 0, 0)


def land(conn):
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0)


def dist_between(lat1, lon1, lat2, lon2):
    dlat = (lat2 - lat1) * 111320
    dlon = (lon2 - lon1) * 111320 * math.cos(math.radians(lat1))
    return math.sqrt(dlat ** 2 + dlon ** 2)

def pose_for_frame(conn, telem, stamp):
    """The vehicle's pose when a frame read at `stamp` was taken: from the telemetry history when there is
    one, else the latest message. None if there is no telemetry at all."""
    pose = telem.at(stamp - CAMERA_LATENCY_S) if telem is not None and stamp else None
    if pose is not None:
        return pose
    gps, att = conn.messages.get("GLOBAL_POSITION_INT"), conn.messages.get("ATTITUDE")
    if gps is None or att is None:
        return None
    return SimpleNamespace(lat=gps.lat / 1e7, lon=gps.lon / 1e7, alt=gps.alt / 1e3, yaw=att.yaw,
                           pitch=att.pitch, roll=att.roll, yawspeed=att.yawspeed)


def locate_boat(name, conn, frame, dets, stamp=None, telem=None):
    """Project the most confident detection onto the water.

    Uses the bbox bottom-centre (where the hull meets the water, which is what the flat
    water plane is intersected with), and the pose at the time the frame was taken (see
    TelemetryLog). Returns (conf, lat, lon, info) with info = {"range" m, "yawspeed" rad/s,
    "roll" rad} describing the pose it was projected from, or (conf, None, None, None) if
    telemetry is missing or the pixel is at/above the horizon.
    """
    conf, (x, y, w, h) = max(dets, key=lambda d: d[0])
    pose = pose_for_frame(conn, telem, stamp)
    if pose is None:
        return conf, None, None, None
    cam = CAMERAS[name]
    img_h, img_w = frame.shape[:2]
    try:
        lat, lon, rng = estimate_from_pixel(
            pose.lat, pose.lon, pose.alt - WATER_AMSL_M,
            math.degrees(pose.yaw), math.degrees(pose.pitch), math.degrees(pose.roll),
            x + w / 2, y + h, img_w, img_h, cam["hfov"], cam["tilt_down_deg"])
    except ValueError:
        return conf, None, None, None
    return conf, lat, lon, {"range": rng, "yawspeed": pose.yawspeed, "roll": pose.roll}


def fix_ok(name, info):
    """Whether a projected fix was taken in a pose good enough to feed the boat track: not too far, and
    not while turning or banking (a small timing error in the attitude is then tens of metres)."""
    return (info is not None and info["range"] <= FIX_MAX_RANGE_M
            and abs(info["yawspeed"]) <= math.radians(FIX_MAX_YAWRATE_DEG_S)
            and abs(info["roll"]) <= math.radians(FIX_MAX_ROLL_DEG[name]))


def feed_track(name, state, lat, lon, info, stamp):
    """Give a fix to the boat track if it is good enough. Returns whether it was."""
    if not fix_ok(name, info):
        return False
    return state.boat_track.update(lat, lon, stamp - CAMERA_LATENCY_S, FIX_WEIGHT[name])


def publish_fix(name, state, lat, lon, info, stamp):
    """A new fix on an already tracked boat. It feeds the track; the raw position is kept as the fallback
    target only until the track has anything better, so a wild fix never replaces a good estimate."""
    used = feed_track(name, state, lat, lon, info, stamp)
    if used or not state.boat_track.has_fix():
        with state.lock:
            state.boat_lat, state.boat_lon = lat, lon


def pick_detection(name, conn, frame, dets, stamp, telem, near=None, gate=None):
    """The detection to treat as the boat: the most confident one that can be placed on the water (a detection
    at or above the horizon cannot be a boat on the sea) and, if `near` is given, lies within `gate` metres of
    it. Returns (detection, lat, lon, info), or None."""
    for det in sorted(dets, key=lambda d: -d[0]):
        _, lat, lon, info = locate_boat(name, conn, frame, [det], stamp, telem)
        if lat is None:
            continue
        if near is not None and dist_between(lat, lon, near[0], near[1]) > gate:
            continue
        return det, lat, lon, info
    return None


def recently_rejected(state, lat, lon, now=None):
    """Is this where a candidate already failed confirmation, lately enough to ignore it?"""
    now = time.time() if now is None else now
    with state.lock:
        state.rejected = [r for r in state.rejected if r[2] > now]
        return any(dist_between(lat, lon, r[0], r[1]) <= REJECT_RADIUS_M for r in state.rejected)


_ignore_logged = {}


def acquire_boat(name, conn, cam, detector, state, pos, tr, telem=None, resume=None):
    """Look for the boat on the latest frame. A detection starts a candidate, which is engaged and confirmed
    (confirm_boat) before anything is committed. Returns True once a boat is found: confirmed by this asset,
    or by another while this one was confirming. `tr` is a new_track() the sighting is recorded in, so whoever
    tracks it next knows which way to turn from the first frame. `resume` re-issues this asset's own goto if a
    candidate was engaged and then rejected (the quad's search loop resends its own every pass)."""
    if detector.model is None:
        return False
    frame, stamp = cam.latest_stamped()
    if frame is None:
        return False
    dets = detector.detect(frame)
    if not dets:
        return False
    pick = pick_detection(name, conn, frame, dets, stamp, telem)
    if pick is None:
        if time.time() - _ignore_logged.get(name, 0.0) > 10:
            _ignore_logged[name] = time.time()
            log(name, "detection ignored: it cannot be placed on the water (at or above the horizon)")
        return False
    det, lat, lon, info = pick
    if recently_rejected(state, lat, lon):
        return False
    return confirm_boat(name, conn, cam, detector, state, tr, telem, resume,
                        first=(frame, det, lat, lon, info, stamp))


def _engage(name, conn, state, tr, cand_track, pos, hits, last_goto):
    """Turn toward the candidate as if engaging it, so it stays in view while it is confirmed. The quad runs the
    same centring servo it uses on a confirmed boat; the plane heads for the candidate, unless that is behind it."""
    if name == "quadcopter":
        follow_boat(conn, tr, cand_track, pos)
        return last_goto
    if time.time() - last_goto < 1.0 or not hits:
        return last_goto
    tlat, tlon = hits[-1][0], hits[-1][1]
    att = conn.messages.get("ATTITUDE")
    if att is not None and abs(wrap_pi(bearing_latlon(pos[:2], (tlat, tlon)) - att.yaw)) > math.radians(CONFIRM_MAX_TURN_DEG):
        return last_goto
    plane_goto(name, conn, tlat, tlon, PLANE_PATROL_ALT)
    return time.time()


def confirm_boat(name, conn, cam, detector, state, tr, telem, resume, first):
    """Engage a candidate and confirm it. Hits are detections that can be placed on the water and lie within
    CONFIRM_GATE_M of the earlier hits' mean; CONFIRM_HITS of them within CONFIRM_WINDOW_FRAMES new frames
    (or CONFIRM_MAX_S) confirm it, and only then is boat_found set. Otherwise it is rejected, that spot is
    ignored for a while, and the caller carries on searching."""
    frame0, det0, lat, lon, info, stamp = first
    hits = [(lat, lon, info, stamp, det0[0])]
    cand_track = BoatTrack()
    if fix_ok(name, info):
        cand_track.update(lat, lon, stamp - CAMERA_LATENCY_S, FIX_WEIGHT[name])
    note_sighting(name, conn, frame0, [det0], tr, stamp, telem)
    log(name, f"possible boat conf={det0[0]:.2f} at ({lat:.5f}, {lon:.5f}): engaging it to confirm "
              f"(need {CONFIRM_HITS} hits in {CONFIRM_WINDOW_FRAMES} frames)")
    frames, seen, start, last_goto = 0, stamp, time.time(), 0.0
    tower_seq0 = state.tower_seq
    while frames < CONFIRM_WINDOW_FRAMES and time.time() - start < CONFIRM_MAX_S and not state.shutdown:
        if state.boat_found:                                  # another asset confirmed it meanwhile
            return True
        if state.tower_seq != tower_seq0 and len(hits) < CONFIRM_TOWER_ABORT_HITS:
            # A tower called while this is still one lone sighting: trust the tower. Not rejected, just left.
            log(name, f"tower call: leaving the candidate ({len(hits)} hit) to respond")
            tr.clear()
            tr.update(new_track())
            if resume is not None:
                resume()
            return False
        pos = get_position(conn)
        if pos:
            with state.lock:
                state.asset_positions[name] = pos
            last_goto = _engage(name, conn, state, tr, cand_track, pos, hits, last_goto)
        new_frame, new_stamp = cam.newest(seen)
        if new_frame is not None:
            seen, frames = new_stamp, frames + 1
            dets = detector.detect(new_frame)
            if dets:
                mean = (sum(h[0] for h in hits) / len(hits), sum(h[1] for h in hits) / len(hits))
                pick = pick_detection(name, conn, new_frame, dets, new_stamp, telem, near=mean, gate=CONFIRM_GATE_M)
                if pick is not None:
                    det, lat, lon, info = pick
                    note_sighting(name, conn, new_frame, [det], tr, new_stamp, telem)
                    hits.append((lat, lon, info, new_stamp, det[0]))
                    if fix_ok(name, info):
                        cand_track.update(lat, lon, new_stamp - CAMERA_LATENCY_S, FIX_WEIGHT[name])
                    if len(hits) >= CONFIRM_HITS:
                        return commit_boat(name, state, tr, hits, frames)
        time.sleep(TRACK_INTERVAL)
    if state.boat_found:
        return True
    mean = (sum(h[0] for h in hits) / len(hits), sum(h[1] for h in hits) / len(hits))
    with state.lock:
        state.rejected.append((mean[0], mean[1], time.time() + REJECT_MEMORY_S))
    log(name, f"not a boat: {len(hits)} agreeing hit(s) in {frames} frames (need {CONFIRM_HITS}); "
              f"ignoring this spot for {REJECT_MEMORY_S:.0f} s and searching on")
    tr.clear()
    tr.update(new_track())                                   # forget where the false candidate sat in the frame
    if resume is not None:
        resume()
    return False


def commit_boat(name, state, tr, hits, frames):
    """The candidate is confirmed: only now does the boat exist for everyone else. The estimate is the mean of
    the hits, and the hits are replayed into the shared track. Returns True."""
    lat = sum(h[0] for h in hits) / len(hits)
    lon = sum(h[1] for h in hits) / len(hits)
    with state.lock:
        if state.boat_found:
            return True                                       # someone else got there first
        state.found_by = name
        state.boat_found = True
        state.boat_lost = False
        state.boat_lat, state.boat_lon = lat, lon
        state.locked_at, state.lock_hits, state.lock_established = time.time(), 0, False   # probation starts now
    tr["hit_at"] = time.time()
    for h in hits:
        feed_track(name, state, h[0], h[1], h[2], h[3])
    log(name, f"BOAT CONFIRMED: {len(hits)} hits in {frames} frames (best conf {max(h[4] for h in hits):.2f}) "
              f"-> ({lat:.5f}, {lon:.5f})")
    return True


def record_tower_call(state, name, tower_num):
    """A tower saw motion. The flags stay set (the plane's tower phase and the status line use them); the call
    is also recorded as a timestamped event so the newest one can win."""
    with state.lock:
        if tower_num == 1:
            state.tower1_detected = True
        else:
            state.tower2_detected = True
        state.tower_seq += 1
        state.last_tower = name
        state.tower_calls[name] = time.time()


def latest_tower_call(state, within=None, now=None):
    """(tower name, when) of the most recent tower call, or None. With `within` seconds: only if it is that recent."""
    now = time.time() if now is None else now
    with state.lock:
        if state.last_tower is None:
            return None
        name, when = state.last_tower, state.tower_calls[state.last_tower]
    if within is not None and now - when > within:
        return None
    return name, when


def check_probation(name, state, now=None):
    """A new lock has to prove itself: PROBATION_HITS hits (from any camera) within PROBATION_S of being
    confirmed, or it is dropped. Returns True if it dropped the boat."""
    now = time.time() if now is None else now
    with state.lock:
        if not state.boat_found or not state.locked_at or state.lock_established:
            return False
        hits = state.lock_hits
        if hits >= PROBATION_HITS:
            state.lock_established = True
            established, expired = True, False
        else:
            established, expired = False, now - state.locked_at > PROBATION_S
    if established:
        log(name, f"lock established: {hits} hits since it was confirmed")
        return False
    if expired:
        drop_boat(name, state, reason=f"only {hits} hit(s) in {PROBATION_S:.0f} s since it was confirmed "
                                      f"(a real boat gives {PROBATION_HITS})")
        return True
    return False


def quad_tower_call(state):
    """Is there a tower call fresh enough for the quad to respond to?"""
    return QUAD_RESPONDS_TO_TOWERS and latest_tower_call(state, within=QUAD_TOWER_ACTIVE_S) is not None


def tower_circle(centre, radius_m, n=8):
    """Corners of a circle around a tower as (lat, lon, heading rad), the heading being the leg leaving each corner."""
    pts = generate_circle_waypoints(centre[0], centre[1], radius_m, n)
    return [(la, lo, bearing_latlon((la, lo), pts[(i + 1) % n])) for i, (la, lo) in enumerate(pts)]


def quad_tower_search(name, conn, cam, detector, state, scan_tr, telem):
    """A tower called: fly to it and search around it on a circle, while the call is fresh (QUAD_TOWER_ACTIVE_S
    after the tower's latest call). The newest call wins: if a different tower calls, go there. Returns when the
    towers have been quiet (the caller resumes the lane search where it stopped), when a boat is found, or on
    shutdown."""
    last_detect, tname, corners, k = 0.0, None, [], 0
    while not state.shutdown and not state.boat_found:
        call = latest_tower_call(state, within=QUAD_TOWER_ACTIVE_S)
        if call is None:
            log(name, "towers quiet: back to the lane search")
            return
        pos = get_position(conn)
        if not pos:
            time.sleep(0.2)
            continue
        with state.lock:
            state.asset_positions[name] = pos
        if call[0] != tname:                                    # the first call, or a different tower called last
            tname = call[0]
            corners = tower_circle(TOWER_POSITIONS[tname], QUAD_TOWER_RADIUS_M)
            k = min(range(len(corners)), key=lambda i: dist_between(pos[0], pos[1], corners[i][0], corners[i][1]))
            log(name, f"TOWER {tname} called: going to search around it ({QUAD_TOWER_RADIUS_M} m circle)")
        wlat, wlon, leg_yaw = corners[k]
        # far from the corner: nose the way we are actually going; on the circle: along the leg
        yaw = bearing_latlon(pos[:2], (wlat, wlon)) if dist_between(pos[0], pos[1], wlat, wlon) > 150 else leg_yaw
        send_goto(conn, wlat, wlon, QUAD_PATROL_ALT, yaw)
        if time.time() - last_detect > DETECTION_INTERVAL:
            last_detect = time.time()
            if acquire_boat(name, conn, cam, detector, state, pos, scan_tr, telem):
                return
        if dist_between(pos[0], pos[1], wlat, wlon) < QUAD_REACH_M:
            k = (k + 1) % len(corners)
        time.sleep(0.2)


def drop_boat(name, state, misses=None, reason=None):
    if reason:
        log(name, f"BOAT LOST — {reason}, resuming search")
    else:
        log(name, f"BOAT LOST — no detection in {misses} frames, resuming search")
    with state.lock:
        state.locked_at, state.lock_hits, state.lock_established = 0.0, 0, False
        state.boat_found = False
        state.locked = False
        state.boat_lost = True
        state.found_by = None
        state.boat_lat = state.boat_lon = 0.0     # never chase a stale estimate
        state.tracker_misses.clear()
    state.boat_track.reset()


def boat_errors(bbox, img_w, img_h, hfov, pitch_rad):
    """How far the boat is from the middle of the frame, as angles (rad): (right of centre, below centre).

    The vertical one is corrected for the copter's own pitch (ATTITUDE, nose up +): leaning forward
    to speed up tilts the fixed camera down and would otherwise make the boat look further away.
    """
    x, y, w, h = bbox
    f = (img_w / 2) / math.tan(hfov / 2)
    return math.atan((x + w / 2 - img_w / 2) / f), math.atan((y + h / 2 - img_h / 2) / f) - pitch_rad


def soft_deadband(err, band):
    """0 inside +-band, then rising from 0 (no jump at the edge)."""
    return 0.0 if abs(err) <= band else err - math.copysign(band, err)


def wrap_pi(a):
    """Angle in radians folded into [-pi, pi)."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def new_track():
    """Fresh tracking record. "stamp"/"misses": frame bookkeeping for track_boat. The rest is the
    last sighting, see note_sighting: "err" (right of, below centre, rad), "box_frac" (boat width
    as a share of the frame), "bearing" (absolute yaw it was seen on, rad), "seen_at" (when)."""
    return {"stamp": 0.0, "misses": 0, "err": None, "box_frac": None, "bearing": None, "seen_at": 0.0}


def note_sighting(name, conn, frame, dets, tr, stamp=None, telem=None, now=None):
    """Record where the most confident detection sits in the frame, and the compass direction it is on
    (from the pose the frame was taken in, see pose_for_frame)."""
    pose = pose_for_frame(conn, telem, stamp)
    if pose is None:
        return
    _, bbox = max(dets, key=lambda d: d[0])
    img_h, img_w = frame.shape[:2]
    yaw_err, tilt_err = boat_errors(bbox, img_w, img_h, CAMERAS[name]["hfov"], pose.pitch)
    tr.update(err=(yaw_err, tilt_err), box_frac=bbox[2] / img_w, bearing=pose.yaw + yaw_err,
              seen_at=time.time() if now is None else now)


def closing_speed_limit(box_frac):
    """Fastest the copter may close in on a boat that fills this share of the frame width: full speed
    while it is small, easing linearly down to FOLLOW_NEAR_MAX_MS once it is big (i.e. near)."""
    if box_frac is None or box_frac <= FOLLOW_SLOW_START_FRAC:
        return FOLLOW_FWD_MAX_MS
    if box_frac >= FOLLOW_SLOW_END_FRAC:
        return FOLLOW_NEAR_MAX_MS
    t = (box_frac - FOLLOW_SLOW_START_FRAC) / (FOLLOW_SLOW_END_FRAC - FOLLOW_SLOW_START_FRAC)
    return FOLLOW_FWD_MAX_MS + t * (FOLLOW_NEAR_MAX_MS - FOLLOW_FWD_MAX_MS)


def follow_command(yaw_err, tilt_err, box_frac=None, tilt_aim=None):
    """(forward m/s, yaw rate rad/s) that walks the boat back to the centre of the frame.

    yaw_err: + = boat right of centre -> turn right. tilt_err: + = boat below centre, i.e. too close
    (it looks steeper) -> back off; - = above centre, too far -> close in. A boat that looks small
    (box_frac under FOLLOW_MIN_BOX_FRAC) is allowed to sit lower in the frame, i.e. we get closer
    to it for a better view, the more so the smaller it is. Closing slows as it looks bigger
    (closing_speed_limit); backing off is not slowed. `tilt_aim` (rad below centre) replaces that
    small-boat allowance with a fixed row to hold: the orbit uses it to keep its radius.
    """
    max_yaw = math.radians(FOLLOW_YAW_MAX_DEG_S)
    aim_below = 0.0
    if tilt_aim is not None:
        aim_below = tilt_aim
    elif box_frac is not None and box_frac < FOLLOW_MIN_BOX_FRAC:
        aim_below = math.radians(FOLLOW_CLOSE_TILT_MAX_DEG) * (1 - box_frac / FOLLOW_MIN_BOX_FRAC)
    yaw_rate = FOLLOW_YAW_KP * soft_deadband(yaw_err, math.radians(FOLLOW_YAW_DEADBAND_DEG))
    forward = -FOLLOW_FWD_KP * soft_deadband(tilt_err - aim_below, math.radians(FOLLOW_TILT_DEADBAND_DEG))
    return (max(-FOLLOW_BACK_MAX_MS, min(closing_speed_limit(box_frac), forward)),
            max(-max_yaw, min(max_yaw, yaw_rate)))


def body_components(ve, vn, yaw):
    """An east/north velocity as (forward, right) for a vehicle heading `yaw` (rad clockwise from north)."""
    return vn * math.cos(yaw) + ve * math.sin(yaw), -vn * math.sin(yaw) + ve * math.cos(yaw)


def limit_speed(forward, right, cap):
    """Scale (forward, right) down, keeping its direction, so the ground speed is at most `cap`."""
    speed = math.hypot(forward, right)
    return (forward, right) if speed <= cap else (forward * cap / speed, right * cap / speed)


def follow_boat(conn, tr, track, pos, now=None):
    """Steer the copter (a body-frame velocity + yaw-rate command, sent here). Returns
    (forward, right, yaw_rate, event); `event` is a line for the log when the phase changed.

    Boat in view: centre it (follow_command). Once its velocity has been estimated (`track`) that
    velocity is added to the command, so the copter travels with the boat instead of trailing it.
    Then, when the boat has been aligned and the range loop has settled for ORBIT_SETTLE_S, the
    orbit begins: a sideways speed plus a yaw rate of speed/radius, nose kept on the boat, radius held
    by keeping the boat on the frame row it had at the start.
    Boat out of view: keep travelling with it, swing toward where the track says it is (else where it
    was last seen) and, if it was last seen small, creep in. The orbit's sideways motion pauses.
    Nothing ever seen -> hover. `pos` is the copter's (lat, lon, ...); `now` is for testing.
    """
    now = time.time() if now is None else now
    att = conn.messages.get("ATTITUDE")
    ve, vn = track.velocity(now)
    ff_forward, ff_right = body_components(ve, vn, att.yaw) if att is not None else (0.0, 0.0)
    boat = track.position(now)
    rng = dist_between(pos[0], pos[1], boat[0], boat[1]) if pos and boat else None
    forward = right = yaw_rate = 0.0
    event = None
    orbit = tr.get("orbit")

    if tr.get("err") is not None and now - tr["seen_at"] <= FOLLOW_STALE_S:
        yaw_err, tilt_err = tr["err"]
        if orbit is not None:
            if not track.converged(now):
                orbit, event = None, "orbit ended: lost the boat's speed estimate"
            elif abs(tilt_err - orbit["tilt0"]) > math.radians(ORBIT_MAX_TILT_DRIFT_DEG):
                orbit, event = None, "orbit ended: the boat drifted out of place in the frame"
            tr["orbit"] = orbit
        p_forward, yaw_rate = follow_command(yaw_err, tilt_err, tr.get("box_frac"),
                                             orbit["tilt0"] if orbit is not None else None)
        if orbit is None:
            settled = (abs(yaw_err) < math.radians(ORBIT_ALIGN_DEG) and abs(p_forward) < ORBIT_SETTLE_MS
                       and track.ready(now) and rng is not None and ORBIT_R_MIN_M <= rng <= ORBIT_R_MAX_M)
            if not settled:
                tr.pop("settled_since", None)
            elif now - tr.setdefault("settled_since", now) >= ORBIT_SETTLE_S:
                orbit = tr["orbit"] = {"tilt0": tilt_err}
                event = (f"ORBIT started: {'clockwise' if ORBIT_DIRECTION > 0 else 'anticlockwise'} at "
                         f"{ORBIT_SPEED_MS:.1f} m/s, radius {rng:.0f} m, {track.describe(now)}")
        forward, right = p_forward + ff_forward, ff_right
        if orbit is not None:
            speed = ORBIT_SPEED_MS * ORBIT_DIRECTION
            right -= speed                              # clockwise, nose on the boat = sideways to the left
            yaw_rate += speed / max(rng or ORBIT_R_MIN_M, ORBIT_R_MIN_M)
    else:
        tr.pop("settled_since", None)
        target = bearing_latlon(pos[:2], boat) if pos and boat else tr.get("bearing")
        if target is not None and att is not None:
            yaw_err = wrap_pi(target - att.yaw)
            yaw_rate = follow_command(yaw_err, 0.0)[1]
            forward, right = ff_forward, ff_right
            # Creep in only if it was last seen small, i.e. far. A boat that was already big and then
            # vanished is more likely just below the bottom of the frame: creeping on would fly over it.
            was_small = (tr.get("box_frac") or 0.0) < FOLLOW_SLOW_START_FRAC
            if orbit is None and was_small and abs(yaw_err) < math.radians(FOLLOW_SEARCH_MAX_YAW_DEG):
                forward += FOLLOW_SEARCH_MS

    max_yaw = math.radians(FOLLOW_YAW_MAX_DEG_S)
    forward, right = limit_speed(forward, right, QUAD_SPEED_MS)
    yaw_rate = max(-max_yaw, min(max_yaw, yaw_rate))
    send_velocity_yawrate(conn, forward, yaw_rate, right)
    return forward, right, yaw_rate, event


def boat_estimate(state):
    """Where the tracked boat is: the filtered track, else the raw fix or the seed. None if there is none yet."""
    tracked = state.boat_track.position(time.time())
    with state.lock:
        blat, blon = tracked if tracked else (state.boat_lat, state.boat_lon)
    return (blat, blon) if blat and blon else None


def should_approach(state, tr, pos):
    """Should this copter fly at the shared estimate rather than centre the boat in its own camera? Yes if its
    camera has never seen the boat, or has not lately and the estimate is still far off (a stray early detection
    must not leave it crawling toward a boat kilometres away). Once a sighting is fresh, or the estimate is
    within JOIN_HANDOFF_M, the centring servo takes over."""
    if tr["err"] is None:
        return True
    if time.time() - tr["seen_at"] <= FOLLOW_STALE_S:
        return False
    est = boat_estimate(state)
    return est is not None and dist_between(pos[0], pos[1], est[0], est[1]) > JOIN_HANDOFF_M


def approach_boat(conn, state, pos):
    """Fly a copter toward the shared estimate, stopping JOIN_STANDOFF_M short with the nose on it: the camera is
    fixed and looks forward, so that puts the boat in front of it. Returns the distance to the estimate, or None
    if there is none yet."""
    est = boat_estimate(state)
    if est is None:
        return None
    blat, blon = est
    d = dist_between(pos[0], pos[1], blat, blon)
    brg = bearing_latlon(pos[:2], (blat, blon))
    if d > JOIN_STANDOFF_M:
        run = d - JOIN_STANDOFF_M
        tlat, tlon = to_latlon(pos[:2], run * math.sin(brg), run * math.cos(brg))
        send_goto(conn, tlat, tlon, QUAD_PATROL_ALT, brg)
    else:
        send_goto(conn, pos[0], pos[1], QUAD_PATROL_ALT, brg)          # close enough: hold and look at it
    return d


def track_boat(name, conn, cam, detector, state, tr, telem=None):
    """One tracking pass for the asset that found the boat.

    Runs inference on the newest camera frame: a detection refreshes the position estimate and
    resets the miss count, no detection counts a miss. After BOAT_LOST_FRAMES misses in a row the
    boat is dropped (BOAT_LOST_FRAMES_PLANE for the plane, whose forward camera loses the boat
    while it orbits). `tr` is a new_track() record; a detection also updates its last sighting
    (see note_sighting). A good fix goes to state.boat_track (publish_fix). A pass with no new
    frame counts nothing, so a stalled camera cannot drop the boat. Detection here uses
    MODEL_CONF_TRACK, a lower bar than finding it took.
    """
    if detector.model is None:
        return
    if check_probation(name, state):                         # a new lock that has not proved itself is forgotten
        return
    frame, tr["stamp"] = cam.newest(tr["stamp"])
    if frame is None:
        return
    dets = detector.detect(frame, conf=MODEL_CONF_TRACK)
    limit = BOAT_LOST_FRAMES_PLANE if name == "fixed-wing" else BOAT_LOST_FRAMES
    pick = None
    if dets:
        # Only a detection near where the track says the boat is counts as the boat: a boat-shaped thing
        # elsewhere (or one that cannot be placed on the water) is a miss, so a false object cannot hold the lock.
        age = min(time.time() - tr["hit_at"], TRACK_HIT_GATE_GROW_MAX_S) if tr.get("hit_at") else 0.0
        pick = pick_detection(name, conn, frame, dets, tr["stamp"], telem, near=boat_estimate(state),
                              gate=TRACK_HIT_GATE_M[name] + TRACK_HIT_GATE_GROW_MS * age)
    if pick is not None:
        det, lat, lon, info = pick
        if tr["misses"]:
            log(name, f"boat reacquired after {tr['misses']} missed frames")
        tr["misses"] = 0
        tr["hit_at"] = time.time()
        with state.lock:
            state.tracker_misses[name] = (0, limit)
            state.lock_hits += 1
        note_sighting(name, conn, frame, [det], tr, tr["stamp"], telem)
        publish_fix(name, state, lat, lon, info, tr["stamp"])
        return
    tr["misses"] += 1
    with state.lock:
        state.tracker_misses[name] = (tr["misses"], limit)
        nobody_sees_it = all(m >= lim for m, lim in state.tracker_misses.values())
    if tr["misses"] >= limit and nobody_sees_it:       # one camera losing it is not enough: another may still have it
        drop_boat(name, state, tr["misses"])
    elif tr["misses"] == 1 or tr["misses"] % 5 == 0:
        log(name, f"boat not seen ({tr['misses']}/{limit} frames)")

# ---------------------------------------------------------------------------
# Waypoint generation
# ---------------------------------------------------------------------------

def corridor(a, b, width, spacing):
    """Corner points (east, north) in metres from `a`: lanes parallel to the line a -> b,
    covering `width` metres centred on it, joined into one back-and-forth path."""
    lat0 = (a[0] + b[0]) / 2
    ex = (b[1] - a[1]) * EARTH_M_PER_DEG * math.cos(math.radians(lat0))
    en = (b[0] - a[0]) * EARTH_M_PER_DEG
    length = math.hypot(ex, en)
    if length < 1.0:
        raise ValueError(f"search corridor endpoints coincide ({a} and {b}): "
                         "the tower positions are wrong (0,0 means the tower had no GPS fix yet)")
    vx, vy = -en / length, ex / length          # unit normal, to the left of a -> b
    lanes = math.ceil(width / spacing) + 1
    pts = []
    for i in range(lanes):
        off = min(-width / 2 + i * spacing, width / 2)
        ends = [(off * vx, off * vy), (ex + off * vx, en + off * vy)]
        pts += ends if i % 2 == 0 else ends[::-1]
    return pts


def bearing(a, b):
    """Radians clockwise from north for a leg a -> b, both (east, north)."""
    return math.atan2(b[0] - a[0], b[1] - a[1]) % (2 * math.pi)


def bearing_latlon(a, b):
    """Radians clockwise from north for a leg a -> b, both (lat, lon)."""
    east = (b[1] - a[1]) * math.cos(math.radians(a[0]))
    return math.atan2(east, b[0] - a[0]) % (2 * math.pi)


def to_latlon(origin, east, north):
    lat0, lon0 = origin
    return (lat0 + north / EARTH_M_PER_DEG,
            lon0 + east / (EARTH_M_PER_DEG * math.cos(math.radians(lat0))))


def lane_waypoints(a, b, width, spacing):
    """(lat, lon, heading rad) per corner; heading is the direction of the leg arriving there."""
    pts = corridor(a, b, width, spacing)
    out = []
    for i, p in enumerate(pts):
        heading = bearing(pts[0], pts[1]) if i == 0 else bearing(pts[i - 1], p)
        out.append((*to_latlon(a, *p), heading))
    return out


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
    returning = False
    try:
        log(name, "connecting...")
        conn = connect(name, wait_ekf=True)
        telem = TelemetryLog(conn)                   # so each frame is projected with the pose it was taken in

        log(name, "setting GUIDED mode")
        set_mode(conn, GUIDED_MODES[name])

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

        set_speed(conn, QUAD_SPEED_MS)

        legs = lane_waypoints(TOWER_POSITIONS[SEARCH_FROM_TOWER], TOWER_POSITIONS[SEARCH_TO_TOWER],
                              SEARCH_WIDTH_M, SEARCH_LANE_SPACING_M)
        # The first leg is the approach from wherever we are, not along a lane: point the nose
        # (and the fixed camera) the way we are actually going, or we fly there backwards.
        here = get_position(conn)
        if here:
            legs[0] = (*legs[0][:2], bearing_latlon(here[:2], legs[0][:2]))

        log(name, f"patrol started — {len(legs)} waypoints, {SEARCH_WIDTH_M}m corridor, "
                  f"{SEARCH_LANE_SPACING_M}m lanes")

        cam = CameraStream(sim_config.camera_url(name))
        last_detect = 0

        n = 0                                        # next lane leg to fly
        scan_tr = new_track()                        # where the boat was first seen, handed to the tracker

        while not state.shutdown:
            # Boat found (by either asset): every asset goes for it. One whose camera has not seen it yet
            # flies to the shared estimate; one that has (the finder, or a joiner once it arrives) keeps
            # it centred in the camera (turn toward it, close in / back off). Each runs its own detection.
            # When no camera has seen it for long enough it is dropped, and the lane search resumes
            # where it stopped.
            if state.boat_found:
                tr = scan_tr                                    # keeps the sighting that found it
                tr["stamp"], tr["misses"] = 0.0, 0
                log(name, "boat found — " +
                          ("centring it in the camera" if state.found_by == name
                           else f"heading for the boat {state.found_by} found"))
                pos = get_position(conn)
                att = conn.messages.get("ATTITUDE")
                if pos:
                    # Stop where we are, not at the lane end, and pin the nose where it points now:
                    # with yaw ignored the autopilot picks its own heading, and aiming at its own
                    # position that heading is arbitrary (a sharp turn).
                    send_goto(conn, pos[0], pos[1], QUAD_PATROL_ALT, att.yaw if att else None)
                if att:
                    log(name, f"heading at lock: {math.degrees(att.yaw) % 360:.0f} deg")
                last_report = 0.0
                while state.boat_found and not state.shutdown:
                    pos = get_position(conn)
                    if pos:
                        with state.lock:
                            state.asset_positions[name] = pos
                        track_boat(name, conn, cam, detector, state, tr, telem)
                        if state.boat_found:                   # track_boat may just have dropped it
                            if should_approach(state, tr, pos):    # no fresh sighting of its own: go where the boat is believed to be
                                d = approach_boat(conn, state, pos)
                                if d is not None and time.time() - last_report > 5:
                                    last_report = time.time()
                                    log(name, f"flying to the boat {state.found_by} found: {d:.0f} m to its estimate")
                            else:
                                forward, right, yaw_rate, event = follow_boat(conn, tr, state.boat_track, pos)
                                if event:
                                    log(name, event)
                                if time.time() - last_report > 2 and tr["err"]:
                                    last_report = time.time()
                                    age = time.time() - tr["seen_at"]
                                    if age <= FOLLOW_STALE_S:
                                        yaw_e, tilt_e = (math.degrees(a) for a in tr["err"])
                                        seen = (f"boat {abs(yaw_e):.1f} deg {'right' if yaw_e >= 0 else 'left'}, "
                                                f"{abs(tilt_e):.1f} deg {'below' if tilt_e >= 0 else 'above'} centre, "
                                                f"{tr['box_frac'] * 100:.1f}% of frame wide")
                                    else:
                                        seen = f"boat not detected for {age:.0f} s, heading for where it should be"
                                    log(name, f"{'[orbit] ' if tr.get('orbit') else ''}{seen}; "
                                              f"{state.boat_track.describe(time.time())} -> {forward:+.1f} m/s fwd, "
                                              f"{right:+.1f} m/s right, turn {math.degrees(yaw_rate):+.1f} deg/s")
                    time.sleep(TRACK_INTERVAL)
                scan_tr = new_track()                           # the next boat starts from a clean record
                continue

            # A tower called, recently enough: go and search around it, then pick the lane search back up.
            if quad_tower_call(state):
                quad_tower_search(name, conn, cam, detector, state, scan_tr, telem)
                continue

            if n >= len(legs):
                log(name, "grid complete — returning to launch")
                set_mode(conn, COPTER_RTL_MODE)
                returning = True
                return

            wlat, wlon, yaw = legs[n]
            while not state.shutdown and not state.boat_found and not quad_tower_call(state):
                send_goto(conn, wlat, wlon, QUAD_PATROL_ALT, yaw)
                pos = get_position(conn)
                if not pos:
                    time.sleep(0.2)
                    continue
                with state.lock:
                    state.asset_positions[name] = pos

                # Keep checking camera while flying
                if time.time() - last_detect > DETECTION_INTERVAL:
                    last_detect = time.time()
                    if acquire_boat(name, conn, cam, detector, state, pos, scan_tr, telem):
                        break

                if dist_between(pos[0], pos[1], wlat, wlon) < QUAD_REACH_M:
                    n += 1
                    break
                time.sleep(0.2)

    except Exception as e:
        log(name, f"error: {crash_reason(e)} — landing")
    finally:
        try:
            if not returning:
                log(name, "landing")
                land(conn)
        except Exception:
            pass


def flight_telemetry(name, conn, state, every=10):
    """Log the autopilot's own flight mode, speeds, climb and bank every `every` s (VFR_HUD, HEARTBEAT, ATTITUDE),
    and every 2 s with a WARNING while it is slow or sinking: the wing stalled once, mid-turn."""
    last_full = 0.0
    while not state.shutdown:
        time.sleep(2)
        hud, hb, att = (conn.messages.get(k) for k in ("VFR_HUD", "HEARTBEAT", "ATTITUDE"))
        if hud is None or hb is None:
            continue
        bank = f" bank={math.degrees(att.roll):+.0f}deg" if att is not None else ""
        line = (f"mode={hb.custom_mode} groundspeed={hud.groundspeed:.1f} m/s "
                f"airspeed(est)={hud.airspeed:.1f} m/s climb={hud.climb:+.1f} m/s throttle={hud.throttle}%{bank}")
        if hud.airspeed < STALL_WARN_AIRSPEED_MS or hud.climb < STALL_WARN_SINK_MS:
            log(name, f"WARNING slow or sinking: {line}")
            last_full = time.time()
        elif time.time() - last_full >= every:
            log(name, f"flight: {line}")
            last_full = time.time()


def run_fixed_wing(state, detector):
    name = "fixed-wing"
    try:
        log(name, "connecting...")
        conn = connect(name, wait_ekf=True)
        telem = TelemetryLog(conn)

        log(name, "setting TAKEOFF mode and arming")
        set_mode(conn, 13)
        if not arm(conn):
            log(name, "ARM FAILED")
            return
        log(name, f"armed — sending takeoff (TAKEOFF mode levels off at {PLANE_TAKEOFF_ALT}m; "
                  f"the patrol then climbs to {PLANE_PATROL_ALT}m)")
        conn.mav.command_long_send(
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, PLANE_TAKEOFF_ALT)

        if not takeoff_and_wait(conn, PLANE_TAKEOFF_ALT, state, timeout=80):
            log(name, "takeoff timed out, continuing anyway")
        else:
            log(name, "reached altitude")

        threading.Thread(target=heartbeat_loop, args=(conn, state), daemon=True).start()

        # ArduPlane only obeys position commands in GUIDED. If the switch is not confirmed the plane
        # keeps orbiting where it took off and every goto is silently ignored, so retry and say so.
        log(name, "switching to GUIDED mode for patrol")
        for attempt in range(1, 6):
            if set_mode(conn, 15):
                log(name, "GUIDED confirmed")
                break
            log(name, f"GUIDED not confirmed (attempt {attempt}/5, plane reports mode "
                      f"{getattr(set_mode, 'last_seen', '?')})")
        else:
            log(name, "WARNING: the plane never entered GUIDED — position commands will be ignored")
        threading.Thread(target=flight_telemetry, args=(name, conn, state), daemon=True).start()

        # The quadcopter's search corridor, flown the other way round: from the tower-2 end. Lanes are
        # laid out to the left of the start -> end line, so swapping the towers also starts on the
        # opposite edge and sweeps the other way. The path wraps to its first waypoint when done.
        waypoints = [(lat, lon) for lat, lon, _ in lane_waypoints(
            TOWER_POSITIONS[SEARCH_TO_TOWER], TOWER_POSITIONS[SEARCH_FROM_TOWER],
            SEARCH_WIDTH_M, SEARCH_LANE_SPACING_M)]
        wp_idx = 0

        log(name, f"patrol started — {len(waypoints)} waypoints, {SEARCH_WIDTH_M}m corridor, "
                  f"{SEARCH_LANE_SPACING_M}m lanes, from the {SEARCH_TO_TOWER} end")

        cam = CameraStream(sim_config.camera_url(name))
        last_detect = 0
        plane_scan_tr = new_track()                  # where a candidate sat in the frame while it was being confirmed

        while not state.shutdown:
            # --- Phase 3: boat found → lock on and follow ---
            if state.boat_found:
                log(name, "BOAT FOUND — locking on to boat" +
                          ("" if state.found_by == name else f" (found by {state.found_by})"))
                state.locked = True
                tr = {"stamp": 0.0, "misses": 0}
                while state.boat_found and not state.shutdown:
                    pos = get_position(conn)
                    if pos:
                        with state.lock:
                            state.asset_positions[name] = pos
                        # Every asset runs its own detection: each sighting feeds the shared track, and the
                        # boat is dropped only when no camera has seen it for BOAT_LOST_FRAMES(_PLANE) frames.
                        track_boat(name, conn, cam, detector, state, tr, telem)

                    # Fly to (and loiter around) the boat's filtered position, extrapolated to now with its
                    # estimated velocity, so the loiter centre follows the boat. The raw fix, or the seed,
                    # only stands in until the track has one.
                    tracked = state.boat_track.position(time.time())
                    with state.lock:
                        blat, blon = tracked if tracked else (state.boat_lat, state.boat_lon)
                    if blat != 0.0 and blon != 0.0:
                        plane_goto(name, conn, blat, blon, PLANE_PATROL_ALT)

                    time.sleep(TRACK_INTERVAL)
                continue        # boat dropped (or shutdown): fall back to tower rush / patrol

            # --- Phase 2: tower detected motion → rush to that area ---
            if state.tower1_detected or state.tower2_detected:
                tname = state.last_tower or ("tower-1" if state.tower1_detected else "tower-2")
                target = TOWER_POSITIONS[tname]

                log(name, f"TOWER {tname} detected motion — orbiting it at {PLANE_TOWER_ORBIT_RADIUS_M} m")
                # One DO_REPOSITION with a loiter radius: the autopilot flies onto a circle around the tower and
                # follows it at a steady, shallow bank. This replaces an 8-waypoint ring that made the plane
                # turn ~120 degrees at each waypoint at ~12 m/s, which stalled it.
                last_sent, last_retarget = 0.0, time.time()
                while (state.tower1_detected or state.tower2_detected) \
                        and not state.boat_found and not state.shutdown:
                    # The newest call wins: swing to the tower that called last (not more often than
                    # TOWER_RETARGET_MIN_S: a 180 degree turn at 12 m/s is a stall risk).
                    if (state.last_tower and state.last_tower != tname
                            and time.time() - last_retarget >= TOWER_RETARGET_MIN_S):
                        tname, last_retarget, last_sent = state.last_tower, time.time(), 0.0
                        target = TOWER_POSITIONS[tname]
                        log(name, f"TOWER {tname} detected motion — now orbiting it at {PLANE_TOWER_ORBIT_RADIUS_M} m")
                    if time.time() - last_sent > 15:            # the target sticks, but say it again in case one was lost
                        last_sent = time.time()
                        plane_goto(name, conn, target[0], target[1], PLANE_PATROL_ALT,
                                   radius=PLANE_TOWER_ORBIT_RADIUS_M)
                    pos = get_position(conn)
                    if not pos:
                        time.sleep(0.5)
                        continue
                    with state.lock:
                        state.asset_positions[name] = pos

                    # Check camera while searching
                    if time.time() - last_detect > DETECTION_INTERVAL:
                        last_detect = time.time()
                        if acquire_boat(name, conn, cam, detector, state, pos, plane_scan_tr, telem,
                                        resume=lambda: plane_goto(name, conn, target[0], target[1], PLANE_PATROL_ALT,
                                                                  radius=PLANE_TOWER_ORBIT_RADIUS_M)):
                            break
                    time.sleep(1)
                continue

            # --- Phase 1: patrol the search corridor ---
            wlat, wlon = waypoints[wp_idx]
            log(name, f"patrol: heading for waypoint {wp_idx + 1}/{len(waypoints)}")
            plane_goto(name, conn, wlat, wlon, PLANE_PATROL_ALT)

            while not state.shutdown and not state.boat_found \
                    and not state.tower1_detected and not state.tower2_detected:
                pos = get_position(conn)
                if not pos:
                    time.sleep(0.5)
                    continue
                with state.lock:
                    state.asset_positions[name] = pos

                if time.time() - last_detect > DETECTION_INTERVAL:
                    last_detect = time.time()
                    if acquire_boat(name, conn, cam, detector, state, pos, plane_scan_tr, telem,
                                    resume=lambda: plane_goto(name, conn, wlat, wlon, PLANE_PATROL_ALT)):
                        break

                d = dist_between(pos[0], pos[1], wlat, wlon)
                if d < WAYPOINT_ARRIVAL_THRESHOLD_M:
                    break
                time.sleep(1)

            wp_idx = (wp_idx + 1) % len(waypoints)

    except Exception as e:
        log(name, f"error: {crash_reason(e)} — landing")
    finally:
        try:
            log(name, "landing")
            land(conn)
        except Exception:
            pass


def settle(tower, state, timeout=20):
    """Wait until the tower has stopped turning (yaw steady for two seconds in a row)."""
    deadline, steady = time.time() + timeout, 0
    time.sleep(1)
    while steady < 2 and time.time() < deadline and not state.shutdown:
        before = tower.yaw
        time.sleep(1)
        steady = steady + 1 if abs(wrap180(tower.yaw - before)) < 0.3 else 0


def sweep_step(tower, direction):
    """Turn the tower TOWER_STEP_DEG along the horizon in `direction` (+1 clockwise, -1 back),
    stopping at the end of its pan range and reversing from there. Returns the new direction."""
    target = tower.pan_us + direction * TOWER_STEP_DEG / tower.gain_pan
    if not 1000 <= target <= 2000 and tower.pan_us in (1000, 2000):
        direction = -direction
    tower.nudge(direction * TOWER_STEP_DEG, 0)
    return direction


def load_tower_range(path, tower_num):
    """The sweep range marked for this tower in tower_aim.py: (left_us, right_us, tilt_us), which
    are exact servo positions. None if the file is missing or the tower has no complete range."""
    try:
        with open(path) as f:
            marks = json.load(f)["sweep"][str(tower_num)]
        marked = tuple(float(marks[k]) for k in ("left_us", "right_us", "tilt_us"))
    except (OSError, KeyError, ValueError, TypeError):
        return None
    return marked if all(1000 <= v <= 2000 for v in marked) else None


def range_stops(left_us, right_us, gain_pan):
    """Pan positions (us) to watch from, evenly spread from the left bound to the right bound and
    no further apart than TOWER_STEP_DEG. A range under a degree wide is watched from one stop."""
    span_deg = abs(right_us - left_us) * abs(gain_pan)
    if span_deg < 1:
        return [(left_us + right_us) / 2]
    n = math.ceil(span_deg / TOWER_STEP_DEG) + 1
    return [left_us + (right_us - left_us) * i / (n - 1) for i in range(n)]


def next_stop(i, direction, count):
    """Move to the next stop in `direction`, turning round at either end. Returns (index, direction)."""
    if count > 1 and not 0 <= i + direction < count:
        direction = -direction
    return (i + direction if count > 1 else 0), direction


def run_tower(name, state, marked):
    tower_num = 1 if name == "tower-1" else 2
    tower = None
    try:
        log(name, "connecting MAVLink...")
        tower = Tower(conn=connect(name))
        time.sleep(2.5)                                   # first attitude messages
        log(name, "calibrating pan/tilt (about 10 s)...")
        tower.calibrate()
        cam = Camera(sim_config.camera_url(name))
        direction = 1
        if marked:
            left_us, right_us, tilt_us = marked
            stops = range_stops(left_us, right_us, tower.gain_pan)
            stop = 0
            tower.set_us(stops[stop], tilt_us)            # start at the left bound, at the marked tilt
            log(name, f"sweeping the marked range: {len(stops)} stops across "
                      f"{abs(right_us - left_us) * abs(tower.gain_pan):.0f} deg")
        else:
            tower.set_us(1500, 1500)                      # start from the centred view
            log(name, "no marked sweep range, sweeping the whole horizon")

        while not state.shutdown:
            settle(tower, state)                          # never learn while the view is moving
            last_stamp = time.time() + 0.5                # skip frames from before it stopped
            watcher = Watcher(tower_num, TOWER_OUT_DIR, learn=TOWER_LEARN_S, sensitivity=16, persist=8)
            log(name, f"new area, bearing {tower.yaw % 360:.0f}: learning the background ({TOWER_LEARN_S:g} s)")
            announced = False
            dwell_end = time.time() + TOWER_LEARN_S + TOWER_WATCH_S

            while not state.shutdown and time.time() < dwell_end:
                if tower.pos:
                    with state.lock:
                        state.asset_positions[name] = tower.pos

                img, last_stamp = cam.newest(last_stamp)
                if img is not None:
                    for fname, bearing, elev, t in watcher.process(img, tower.yaw, tower.pitch):
                        log(name, "MOTION DETECTED! " + describe(t, bearing, elev, fname))
                        record_tower_call(state, name, tower_num)
                if not watcher.learning and not announced:
                    log(name, "watching for movement")
                    announced = True
                time.sleep(0.2)

            if not state.shutdown:
                if marked:
                    stop, direction = next_stop(stop, direction, len(stops))
                    tower.set_us(stops[stop], tilt_us)
                else:
                    direction = sweep_step(tower, direction)
                log(name, "turning to a new area")

    except (Exception, SystemExit) as e:                  # calibrate() exits on failure
        log(name, f"error: {e}")
    finally:
        if tower is not None:
            tower.pan_us = tower.tilt_us = None           # release; the tower drifts back to centre


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
        if state.boat_lost and not state.boat_found:
            flags.append("BOAT_LOST")

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

def discover_tower_positions():
    """Ask each tower where it is (local sim only; there they are placed by the sim's .env).

    Raises ConnectionError rather than falling back to the remote coordinates, which would
    send the drones to places that are not where the local towers are.
    """
    found = {}
    for name in TOWER_POSITIONS:
        gps, seen = None, False
        try:
            conn = connect(name)
            # A tower that has just (re)started reports lat/lon 0,0 until it has a fix: wait it out.
            deadline = time.time() + TOWER_FIX_TIMEOUT_S
            while time.time() < deadline:
                msg = conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
                if msg is None:
                    continue
                seen = True
                if msg.lat or msg.lon:
                    gps = msg
                    break
            conn.close()
        except Exception as e:
            raise ConnectionError(f"could not reach {name} at {sim_config.mavlink_url(name)}: {e}")
        if not gps:
            if seen:
                raise ConnectionError(f"{name} still reports position 0,0 after {TOWER_FIX_TIMEOUT_S:.0f} s: "
                                      "the local sim has not finished starting, wait and rerun (./mavcheck)")
            raise ConnectionError(f"{name} sent no position; is the local sim up? (./mavcheck)")
        found[name] = (gps.lat / 1e7, gps.lon / 1e7)
    return found


def publish_boat(state):
    """Send the filtered boat position to the WebXR map. Silent while no boat is tracked, so its pin fades."""
    while not state.shutdown:
        if state.boat_found:
            tracked = state.boat_track.position(time.time())
            lat, lon = tracked if tracked else (state.boat_lat, state.boat_lon)
            if lat or lon:
                _WEBXR_FEED.update("boat", "boat", lat, lon, 0.0)
        time.sleep(0.5)


def main():
    parser = argparse.ArgumentParser(description="Coordinated drone mission for Arctic SIM-8")
    sim_config.add_argument(parser)
    parser.add_argument("--tower-config", default=AIM_FILE,
                        help="tower_aim.json holding the sweep ranges marked with tower_aim.py "
                             f"(default: {AIM_FILE})")
    parser.add_argument("--webxr-feed", nargs="?", const=8781, type=int, metavar="PORT",
                        help="serve asset and boat positions to the WebXR map on this port (default 8781)")
    args = parser.parse_args()
    sim_config.configure(args)

    if args.webxr_feed:
        global _WEBXR_FEED
        sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "webxr", "bridge"))
        import feed as webxr_feed
        _WEBXR_FEED = webxr_feed.Feed()
        webxr_feed.serve(_WEBXR_FEED, args.webxr_feed)
        print(f"WebXR map feed on http://0.0.0.0:{args.webxr_feed}/positions")

    print("=" * 60)
    print("ARCTIC SIM-8 — COORDINATED MISSION")
    print("=" * 60)
    print()
    print(f"Sim: {sim_config.target()} ({sim_config.host()})")
    if sim_config.is_local():
        try:
            TOWER_POSITIONS.update(discover_tower_positions())
        except ConnectionError as e:
            sys.exit(f"ERROR: {e}")
        for name, (lat, lon) in TOWER_POSITIONS.items():
            print(f"  {name} at ({lat:.6f}, {lon:.6f})")
    tower_ranges = {}
    for num, tname in enumerate(("tower-1", "tower-2"), 1):
        tower_ranges[tname] = load_tower_range(args.tower_config, num)
        print(f"  {tname} sweep: " + ("the range marked in " + os.path.basename(args.tower_config)
              if tower_ranges[tname] else f"NO RANGE MARKED in {args.tower_config}, using the whole horizon"))
    print("Assets: quadcopter, fixed-wing, tower-1, tower-2")
    print("AI: YOLO boat detection + tower motion detection")
    print("Ctrl+C to land all drones and exit")
    print()

    state = SharedState()
    detector = BoatDetector(MODEL_PATH, conf=MODEL_CONF)

    threads = [
        threading.Thread(target=run_quadcopter, args=(state, detector), name="quadcopter"),
        threading.Thread(target=run_fixed_wing, args=(state, detector), name="fixed-wing"),
        threading.Thread(target=run_tower, args=("tower-1", state, tower_ranges["tower-1"]), name="tower-1"),
        threading.Thread(target=run_tower, args=("tower-2", state, tower_ranges["tower-2"]), name="tower-2"),
        threading.Thread(target=status_printer, args=(state,), name="status"),
    ]
    if _WEBXR_FEED is not None:
        threads.append(threading.Thread(target=publish_boat, args=(state,), name="webxr-boat"))

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
