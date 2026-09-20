"""Watch from a camera tower with background subtraction and flag things that move.

A tower camera never moves once aimed, so everything static (land, ice, water, sky) is
learned as background and only moving objects, like a vessel under way, stand out.

    python3 tower_watch.py --tower 2 --pan 118 --tilt -4.4     # aim, learn, then watch
    python3 tower_watch.py --tower 2                            # watch wherever it points now
    python3 tower_watch.py --tower 2 --sim local                # against your own docker compose sim

What it does:
  1. (--pan/--tilt) aims the tower over MAVLink. The tracker is in MANUAL mode, where RC
     channel 1 is pan and channel 2 is tilt, so it sends RC overrides continuously. The
     PWM-to-degrees scale and direction are measured by probing, not assumed. --pan is a
     compass bearing in degrees, --tilt is degrees above the horizon (negative = down).
  2. Learns the background for --learn seconds. Keep the aim fixed and don't expect a
     moving vessel to be flagged during this time.
  3. Flags blobs that stay put in the "moving" mask for --persist consecutive looks, saves
     the frame plus a magnified crop, and prints an alert with an estimated bearing.

Stop with Ctrl-C. Needs pymavlink, opencv and numpy.
"""
import argparse
import csv
import json
import math
import os
import sys
import threading
import time

import cv2
import numpy as np

import sim_config

try:
    from pymavlink import mavutil
except ImportError:
    mavutil = None

# tower number -> (MAVLink GCS ports to try, camera stream port). From the sim README and
# terrain/tower.py; override with --connect / --camera.
TOWERS = {1: ([14580, 14581], 8630), 2: ([14590, 14591], 8640)}
AIM_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "tower_aim.json")
HFOV = 1.047                      # radians, the tower EO camera's horizontal field of view
WIDTH = 1280


def wrap180(deg):
    return (deg + 180) % 360 - 180


class Camera:
    """Newest frame from the MJPEG stream, read in a thread (the sim only refreshes its
    frame while a /stream client is connected, so snapshots go stale)."""

    def __init__(self, url):
        self.url, self._frame, self._stamp = url, None, 0.0
        self._lock = threading.Lock()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while True:
            cap = cv2.VideoCapture(self.url)
            while cap.isOpened():
                ok, frame = cap.read()
                if not ok:
                    break
                with self._lock:
                    self._frame, self._stamp = frame, time.time()
            cap.release()
            time.sleep(1)

    def latest(self, max_age=2.0):
        with self._lock:
            fresh = self._frame is not None and time.time() - self._stamp <= max_age
            return self._frame.copy() if fresh else None

    def newest(self, after, max_age=2.0):
        """(frame, stamp) if a frame newer than `after` has arrived, else (None, after).
        The stream delivers only a few frames a second, so this lets the caller process
        each frame once instead of re-counting the same one."""
        with self._lock:
            if self._frame is None or self._stamp <= after or time.time() - self._stamp > max_age:
                return None, after
            return self._frame.copy(), self._stamp


class Tower:
    """MAVLink link to a camera tower's AntennaTracker: reads attitude, holds an aim."""

    def __init__(self, urls=(), conn=None):
        """Connect to the first of `urls` that answers, or take over an open `conn` (which this
        object then owns: nothing else may read from it)."""
        if mavutil is None:
            sys.exit("pymavlink is not installed: pip install pymavlink")
        self.m = conn
        for url in urls if conn is None else ():
            m = mavutil.mavlink_connection(url)
            m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            if m.wait_heartbeat(timeout=6):
                self.m = m
                break
        if self.m is None:
            sys.exit(f"no heartbeat from the tower on {urls}; try ./mavcheck tower-N in the sim folder")
        self.yaw = self.pitch = None
        self.pos = None                            # (lat, lon, height above spawn) once reported
        self.pan_us = self.tilt_us = None          # None = not overriding
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        last_hb = last_rc = 0.0
        while True:
            msg = self.m.recv_match(blocking=True, timeout=0.1)
            kind = msg.get_type() if msg is not None else None
            if kind == "ATTITUDE":
                self.yaw, self.pitch = math.degrees(msg.yaw), math.degrees(msg.pitch)
            elif kind == "GLOBAL_POSITION_INT":
                self.pos = (msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1e3)
            now = time.time()
            if now - last_hb > 1:
                self.m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                last_hb = now
            if self.pan_us is not None and now - last_rc > 0.2:
                # 65535 = leave that channel alone. The override expires within seconds if
                # it isn't refreshed, so this has to keep running for the aim to hold.
                self.m.mav.rc_channels_override_send(
                    self.m.target_system, self.m.target_component,
                    int(self.pan_us), int(self.tilt_us), 65535, 65535, 65535, 65535, 65535, 65535)
                last_rc = now

    def _read(self, wait=4.0):
        time.sleep(wait)
        if self.yaw is None:
            sys.exit("no ATTITUDE from the tower")
        return self.yaw, self.pitch

    def calibrate(self):
        """Measure degrees per microsecond on each axis by probing, so no scale or sign is
        assumed. Starts from the centred pose (1500/1500) and ends at 1600/1600."""
        self.pan_us = self.tilt_us = 1500
        y0, p0 = self._read()
        self.pan_us = self.tilt_us = 1600
        y1, p1 = self._read()
        self.gain_pan, self.gain_tilt = wrap180(y1 - y0) / 100.0, (p1 - p0) / 100.0
        if abs(self.gain_pan) < 0.01 or abs(self.gain_tilt) < 0.01:
            sys.exit("the tower did not respond to pan/tilt overrides (is it in MANUAL mode?)")
        print(f"  measured: pan {self.gain_pan:+.3f} deg/us, tilt {self.gain_tilt:+.3f} deg/us")

    def nudge(self, d_bearing, d_tilt):
        """Move the aim by degrees: +bearing turns clockwise, +tilt looks up."""
        self.pan_us = min(2000, max(1000, self.pan_us + d_bearing / self.gain_pan))
        self.tilt_us = min(2000, max(1000, self.tilt_us + d_tilt / self.gain_tilt))

    def set_us(self, pan_us, tilt_us):
        """Command the servo positions directly. Unlike a compass bearing (an estimate that
        can wander a few degrees), these reproduce exactly the same camera direction."""
        self.pan_us, self.tilt_us = pan_us, tilt_us

    def at_limit(self):
        return self.pan_us in (1000, 2000) or self.tilt_us in (1000, 2000)

    def aim(self, bearing, tilt):
        """Point the camera at compass `bearing` and `tilt` degrees above horizontal."""
        self.calibrate()
        y, p = self.yaw, self.pitch
        for _ in range(3):
            self.nudge(wrap180(bearing - y), tilt - p)
            y, p = self._read()
            if abs(wrap180(bearing - y)) < 0.5 and abs(tilt - p) < 0.5:
                break
        return y, p


def static_edges(bg_img, grad_thresh=30, grow=7):
    """Mask of pixels on or near hard edges in the learned background (ridgelines against
    the sky, coast, ice edges). Render differences make these flicker from frame to frame
    and they would otherwise look like motion."""
    gray = cv2.cvtColor(bg_img, cv2.COLOR_BGR2GRAY)
    gx, gy = cv2.Sobel(gray, cv2.CV_16S, 1, 0), cv2.Sobel(gray, cv2.CV_16S, 0, 1)
    edges = (np.abs(gx) + np.abs(gy)) > grad_thresh * 4          # a step of s levels gives ~4*s
    return cv2.dilate(edges.astype(np.uint8) * 255, np.ones((grow, grow), np.uint8))


def update_tracks(tracks, blobs, now, gate=25.0):
    """Match this look's blobs to existing tracks by distance; unmatched blobs start new tracks."""
    for blob in blobs:
        best = min(tracks, key=lambda t: math.hypot(t["x"] - blob[0], t["y"] - blob[1]), default=None)
        if best and math.hypot(best["x"] - blob[0], best["y"] - blob[1]) <= gate:
            best.update(x=blob[0], y=blob[1], box=blob[2:], hits=best["hits"] + 1, last=now)
        else:
            tracks.append(dict(x=blob[0], y=blob[1], box=blob[2:], hits=1, last=now, alerted=False))
    return [t for t in tracks if now - t["last"] <= 3.0]


def pixel_to_angles(x, y, h):
    f = (WIDTH / 2) / math.tan(HFOV / 2)
    return math.degrees(math.atan((x - WIDTH / 2) / f)), math.degrees(math.atan((h / 2 - y) / f))


def save_alert(out_dir, tower_no, img, track, yaw, pitch):
    os.makedirs(out_dir, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    x, y, w, h, area = track["box"]
    cx, cy = int(track["x"]), int(track["y"])
    marked = img.copy()
    cv2.rectangle(marked, (int(x) - 12, int(y) - 12), (int(x + w) + 12, int(y + h) + 12), (0, 255, 0), 2)
    r = 60
    pad = cv2.copyMakeBorder(img, r, r, r, r, cv2.BORDER_CONSTANT, value=(0, 0, 0))
    crop = cv2.resize(pad[cy:cy + 2 * r, cx:cx + 2 * r], None, fx=5, fy=5, interpolation=cv2.INTER_CUBIC)
    base = os.path.join(out_dir, f"{stamp}_tower{tower_no}")
    cv2.imwrite(base + ".png", img)
    cv2.imwrite(base + "_annotated.png", marked)
    cv2.imwrite(base + "_crop.png", crop)
    az, el = pixel_to_angles(track["x"], track["y"], img.shape[0])
    bearing = (yaw + az) % 360 if yaw is not None else float("nan")
    log = os.path.join(out_dir, "detections.csv")
    new = not os.path.exists(log)
    with open(log, "a", newline="") as f:
        wr = csv.writer(f)
        if new:
            wr.writerow(["time", "tower", "file", "x", "y", "w", "h", "est_bearing_deg", "est_elevation_deg"])
        wr.writerow([stamp, tower_no, os.path.basename(base) + ".png", cx, cy, int(w), int(h),
                     f"{bearing:.1f}", f"{(pitch or 0) + el:.1f}"])
    return os.path.basename(base), bearing, (pitch or 0) + el


class Watcher:
    """Background subtraction on a camera that stays still: learn the background for `learn`
    seconds, then flag blobs that persist. Feed it every new frame through process()."""

    def __init__(self, tower_no, out_dir, learn=30, sensitivity=16, rate=0.0005,
                 min_area=30, max_area=4000, persist=8):
        self.tower_no, self.out_dir = tower_no, out_dir
        self.learn, self.rate = learn, rate
        self.min_area, self.max_area, self.persist = min_area, max_area, persist
        self.bg = cv2.createBackgroundSubtractorMOG2(history=500, varThreshold=sensitivity, detectShadows=False)
        self.tracks, self.edges = [], None
        self.looks = self.alerts = 0
        self.started = time.time()

    @property
    def learning(self):
        return time.time() - self.started < self.learn

    def learn_left(self):
        return max(0.0, self.learn - (time.time() - self.started))

    def process(self, img, yaw=None, pitch=None):
        """Feed one new frame. Returns [(saved name, est. bearing, est. elevation, track)] for
        objects newly confirmed as moving; empty while still learning."""
        learning = self.learning
        mask = self.bg.apply(cv2.GaussianBlur(img, (3, 3), 0), learningRate=0.05 if learning else self.rate)
        if learning:
            return []
        if self.edges is None or self.looks % 300 == 0:
            self.edges = static_edges(self.bg.getBackgroundImage())
        mask = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)[1]
        mask = cv2.dilate(cv2.bitwise_and(mask, cv2.bitwise_not(self.edges)), np.ones((5, 5), np.uint8))
        n, _, stats, cents = cv2.connectedComponentsWithStats(mask)
        blobs = [(cents[i][0], cents[i][1], *stats[i][:5]) for i in range(1, n)
                 if self.min_area <= stats[i][cv2.CC_STAT_AREA] <= self.max_area]
        self.tracks = update_tracks(self.tracks, blobs, time.time())
        self.looks += 1
        found = []
        for t in self.tracks:
            if t["hits"] >= self.persist and not t["alerted"]:
                t["alerted"] = True
                self.alerts += 1
                name, bearing, elev = save_alert(self.out_dir, self.tower_no, img, t, yaw, pitch)
                found.append((name, bearing, elev, t))
        return found


def describe(t, bearing, elev, name):
    return (f"*** MOVING OBJECT  pixel ({t['x']:.0f}, {t['y']:.0f})  size {t['box'][2]}x{t['box'][3]}  "
            f"est. bearing {bearing:.1f}  elevation {elev:+.1f}  -> {name}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--tower", type=int, choices=[1, 2], required=True)
    ap.add_argument("--pan", type=float, help="compass bearing to aim at, degrees")
    ap.add_argument("--tilt", type=float, help="degrees above horizontal (negative = look down)")
    ap.add_argument("--saved", action="store_true", help="use the aim saved by tower_aim.py")
    sim_config.add_argument(ap)
    ap.add_argument("--connect", help="MAVLink endpoint (default: the tower's GCS port)")
    ap.add_argument("--camera", help="camera stream URL (default: the tower's port)")
    ap.add_argument("--learn", type=float, default=30, help="seconds to learn the background")
    ap.add_argument("--sensitivity", type=float, default=16, help="MOG2 variance threshold; lower = more sensitive")
    ap.add_argument("--rate", type=float, default=0.0005, help="background learning rate while watching")
    ap.add_argument("--min-area", type=int, default=30, help="smallest blob, px^2, after dilation")
    ap.add_argument("--max-area", type=int, default=4000)
    ap.add_argument("--persist", type=int, default=8, help="looks a blob must persist before alerting")
    ap.add_argument("--seconds", type=float, default=0, help="stop after this long (0 = until Ctrl-C)")
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "tower_output"))
    args = ap.parse_args()
    sim_config.configure(args)

    saved_us = None
    if args.saved:
        try:
            saved = json.load(open(AIM_FILE))[str(args.tower)]
        except (OSError, KeyError, ValueError):
            sys.exit(f"no saved aim for tower {args.tower}; run tower_aim.py --tower {args.tower} and press s")
        args.pan, args.tilt = saved["pan"], saved["tilt"]
        saved_us = (saved["pan_us"], saved["tilt_us"]) if "pan_us" in saved else None
    ports, cam_port = TOWERS[args.tower]
    urls = [args.connect] if args.connect else [f"udpout:{sim_config.host()}:{p}" for p in ports]
    tower = Tower(urls)
    if (args.pan is None) != (args.tilt is None):
        sys.exit("give --pan and --tilt together, or neither")
    if saved_us:
        print(f"restoring the saved aim (pan {saved_us[0]:.0f} us, tilt {saved_us[1]:.0f} us) ...")
        tower.set_us(*saved_us)
        time.sleep(5)
        print(f"  now at bearing {tower.yaw % 360:.1f} (estimate), tilt {tower.pitch:+.1f}")
    elif args.pan is not None:
        print(f"aiming tower {args.tower} at bearing {args.pan:.1f}, tilt {args.tilt:+.1f} ...")
        yaw, pitch = tower.aim(args.pan, args.tilt)
        print(f"  now at bearing {yaw % 360:.1f}, tilt {pitch:+.1f}")
    camera = Camera(args.camera or f"http://{sim_config.host()}:{cam_port}/stream")

    watcher = Watcher(args.tower, args.out, learn=args.learn, sensitivity=args.sensitivity, rate=args.rate,
                      min_area=args.min_area, max_area=args.max_area, persist=args.persist)
    start, last_note, last_stamp, announced = time.time(), 0.0, 0.0, False
    print(f"MODE: LEARNING the background for {args.learn:.0f} s. Keep the view still; nothing is flagged yet.")
    try:
        while True:
            if args.seconds and time.time() - start > args.seconds:
                break
            img, last_stamp = camera.newest(last_stamp)
            if img is None:
                time.sleep(0.05)
                continue
            for name, bearing, elev, t in watcher.process(img, tower.yaw, tower.pitch):
                print("\a" + describe(t, bearing, elev, name))
            if watcher.learning:
                if time.time() - last_note > 10:
                    print(f"  learning: {watcher.learn_left():.0f} s left")
                    last_note = time.time()
            elif not announced:
                print("MODE: WATCHING. Anything that moves will be flagged here, with a beep.")
                announced = True
            elif time.time() - last_note > 10:
                print(f"  watching: {watcher.looks} frames checked, {len(watcher.tracks)} candidates, {watcher.alerts} alerts")
                last_note = time.time()
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        tower.pan_us = tower.tilt_us = None      # stop overriding; the tower drifts back to centre
        print("stopped")


if __name__ == "__main__":
    main()
