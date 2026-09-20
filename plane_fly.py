#!/usr/bin/env python3
"""
Fly the fixed-wing on its own — no towers, cameras or mission logic — to check that it goes where it is told.

What ArduPlane 4.5 (the source the sim builds) says about GUIDED mode:
  * The plane flies to the guided target and then orbits it at WP_LOITER_RAD (120 m in plane.parm).
  * Entering GUIDED sets the target to where the plane is *right now* (mode_guided.cpp, _enter), so a plane
    that switched to GUIDED and was never given a target just circles where it switched.
  * SET_POSITION_TARGET_GLOBAL_INT is only honoured for ALTITUDE on ArduPlane (GCS_Mavlink.cpp, the
    MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT case): latitude and longitude are never read. It is what a
    copter is sent to fly somewhere, and on the plane it makes it climb and nothing else.
  * MAV_CMD_DO_REPOSITION is the way to send a plane somewhere. It sets the guided target (lat, lon, alt) and
    the loiter radius, can switch to GUIDED itself, and is acknowledged with a COMMAND_ACK. Sent as
    COMMAND_INT the altitude frame is explicit (MAV_FRAME_GLOBAL_RELATIVE_ALT = metres above home).

Usage:
    python3 plane_fly.py --sim local                   # take off, fly a 3-point triangle, then return home
    python3 plane_fly.py --sim local --method target   # the copter-style command, to watch the plane ignore it
    python3 plane_fly.py --sim local --no-takeoff      # the plane is already flying
    python3 plane_fly.py --sim local --distance 800 --legs 2

It prints one line every 2 s: the plane's mode, how far it is from the target, its speed and heading (with the
bearing it *should* be flying), and what the autopilot itself says its target is. Ctrl+C switches to RTL.
"""

import argparse
import math
import sys
import time

from pymavlink import mavutil

import sim_config

M_PER_DEG = 111_320.0
MODE_RTL, MODE_TAKEOFF, MODE_GUIDED = 11, 13, 15        # ArduPlane custom modes
ACK = {0: "ACCEPTED", 1: "TEMPORARILY_REJECTED", 2: "DENIED", 3: "UNSUPPORTED", 4: "FAILED", 5: "IN_PROGRESS"}
MODE_NAMES = {0: "MANUAL", 1: "CIRCLE", 2: "STABILIZE", 5: "FBWA", 6: "FBWB", 7: "CRUISE", 10: "AUTO",
              11: "RTL", 12: "LOITER", 13: "TAKEOFF", 15: "GUIDED"}


def say(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------

def distance_m(a, b):
    dlat = (b[0] - a[0]) * M_PER_DEG
    dlon = (b[1] - a[1]) * M_PER_DEG * math.cos(math.radians(a[0]))
    return math.hypot(dlat, dlon)


def bearing_deg(a, b):
    east = (b[1] - a[1]) * math.cos(math.radians(a[0]))
    return math.degrees(math.atan2(east, b[0] - a[0])) % 360


def offset(origin, dist, bearing):
    lat, lon = origin
    return (lat + dist * math.cos(math.radians(bearing)) / M_PER_DEG,
            lon + dist * math.sin(math.radians(bearing)) / (M_PER_DEG * math.cos(math.radians(lat))))


# ---------------------------------------------------------------------------
# MAVLink plumbing
# ---------------------------------------------------------------------------

def gcs_heartbeat(conn):
    conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)


def pump(conn):
    """Read everything that is queued. Messages arrive faster than a slow loop reads them one at a time, and a
    single read returns the OLDEST queued message, so without this the plane's position runs further behind
    every second. After pump(), conn.messages holds the newest of each type."""
    while conn.recv_match(blocking=False) is not None:
        pass


def connect():
    url = sim_config.mavlink_url("fixed-wing")
    say(f"connecting to the plane on {url}")
    conn = mavutil.mavlink_connection(url, source_system=255)
    for _ in range(5):
        gcs_heartbeat(conn)
        time.sleep(0.2)
    conn.mav.request_data_stream_send(0, 0, mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
    hb = conn.wait_heartbeat(timeout=8)
    if hb is None:
        sys.exit("no heartbeat from the plane. Is the sim up, and did you pass --sim local? (./mavcheck in the sim folder)")
    say(f"heartbeat from system {conn.target_system} component {conn.target_component}, "
        f"mode {mode_name(hb.custom_mode)}, armed={bool(hb.base_mode & 128)}")
    return conn


def mode_name(m):
    return f"{MODE_NAMES.get(m, '?')}({m})"


def snapshot(conn):
    """Pump the queue and return the newest position/speed/mode data, or None until there is a real fix."""
    pump(conn)
    gps, hb, hud = conn.messages.get("GLOBAL_POSITION_INT"), conn.messages.get("HEARTBEAT"), conn.messages.get("VFR_HUD")
    if gps is None or hb is None or (gps.lat == 0 and gps.lon == 0):
        return None
    return {"pos": (gps.lat / 1e7, gps.lon / 1e7), "alt": gps.relative_alt / 1e3, "mode": hb.custom_mode,
            "armed": bool(hb.base_mode & 128), "hdg": gps.hdg / 100 if gps.hdg != 65535 else None,
            "gs": hud.groundspeed if hud is not None else float("nan"),
            "tgt": conn.messages.get("POSITION_TARGET_GLOBAL_INT"), "nav": conn.messages.get("NAV_CONTROLLER_OUTPUT")}


def wait_for(conn, what, done, timeout, every=5):
    """Poll snapshot() until done(snap) is true. Sends GCS heartbeats, prints a progress line every `every` s."""
    start, last = time.time(), 0
    while time.time() - start < timeout:
        gcs_heartbeat(conn)
        s = snapshot(conn)
        if s and done(s):
            return s
        if time.time() - last >= every:
            last = time.time()
            say(f"  waiting for {what}... " + (f"mode {mode_name(s['mode'])} alt {s['alt']:.0f} m" if s else "no position yet"))
        time.sleep(0.5)
    return None


def set_mode(conn, mode, tries=5):
    for attempt in range(1, tries + 1):
        conn.mav.set_mode_send(conn.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode)
        s = wait_for(conn, f"mode {mode_name(mode)}", lambda s: s["mode"] == mode, timeout=3, every=99)
        if s:
            say(f"mode {mode_name(mode)} confirmed")
            return True
        say(f"mode {mode_name(mode)} not confirmed (attempt {attempt}/{tries})")
    return False


def wait_ack(conn, command, timeout=3):
    end = time.time() + timeout
    while time.time() < end:
        msg = conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
        if msg is not None and msg.command == command:
            return msg.result
    return None


# ---------------------------------------------------------------------------
# Starting up
# ---------------------------------------------------------------------------

def start_up(conn, alt):
    """Take off in TAKEOFF mode. It climbs to the TKOFF_ALT parameter (60 m in plane.parm) and holds there, whatever
    altitude NAV_TAKEOFF asks for, so `alt` is where we expect it to level off, not where the mission will fly."""
    say("waiting for the EKF to settle (up to 60 s)")
    end = time.time() + 60
    while time.time() < end:
        gcs_heartbeat(conn)
        m = conn.recv_match(blocking=True, timeout=1)
        if m is not None and m.get_type() == "EKF_STATUS_REPORT" and m.flags & 0x1F == 0x1F:
            say("EKF ready")
            break
    else:
        say("EKF wait timed out, carrying on")
    if not snapshot(conn):
        wait_for(conn, "a GPS position", lambda s: True, timeout=30)

    say("TAKEOFF mode, arming")
    if not set_mode(conn, MODE_TAKEOFF):
        sys.exit("the plane would not enter TAKEOFF mode")
    for attempt in range(1, 6):
        conn.mav.command_long_send(conn.target_system, conn.target_component,
                                   mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        if wait_for(conn, "armed", lambda s: s["armed"], timeout=5, every=99):
            break
        say(f"not armed yet (attempt {attempt}/5)")
    else:
        sys.exit("the plane would not arm")
    say(f"armed, taking off (TAKEOFF mode climbs to TKOFF_ALT, expected about {alt:.0f} m)")
    conn.mav.command_long_send(conn.target_system, conn.target_component,
                               mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt)
    start, last, history = time.time(), 0, []
    while time.time() - start < 240:
        gcs_heartbeat(conn)
        s = snapshot(conn)
        if s:
            history.append((time.time(), s["alt"]))
            if s["alt"] >= alt * 0.9:
                say(f"at {s['alt']:.0f} m")
                return
            old = [a for t, a in history if time.time() - t >= 20]            # altitude 20+ s ago
            if s["alt"] > 30 and old and s["alt"] - old[-1] < 1.5:
                say(f"levelled off at {s['alt']:.0f} m (TAKEOFF mode holds at TKOFF_ALT): carrying on")
                return
        if time.time() - last >= 5:
            last = time.time()
            say(f"  climbing... " + (f"alt {s['alt']:.0f} m" if s else "no position yet"))
        time.sleep(0.5)
    say("takeoff did not finish in time, carrying on from wherever it is")


# ---------------------------------------------------------------------------
# The two ways of asking the plane to go somewhere
# ---------------------------------------------------------------------------

def goto_reposition(conn, lat, lon, alt, radius=0):
    """MAV_CMD_DO_REPOSITION as COMMAND_INT. Returns the COMMAND_ACK result name, or None if none came."""
    conn.mav.command_int_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,               # altitude = metres above home
        mavutil.mavlink.MAV_CMD_DO_REPOSITION, 0, 0,
        -1,                                                          # param1 ground speed, -1 = default
        mavutil.mavlink.MAV_DO_REPOSITION_FLAGS_CHANGE_MODE,         # param2 switch to GUIDED if needed
        radius,                                                      # param3 loiter radius, 0 = WP_LOITER_RAD
        float("nan"),                                                # param4 NaN = clockwise
        int(lat * 1e7), int(lon * 1e7), alt)
    result = wait_ack(conn, mavutil.mavlink.MAV_CMD_DO_REPOSITION)
    return None if result is None else ACK.get(result, str(result))


def goto_target(conn, lat, lon, alt):
    """SET_POSITION_TARGET_GLOBAL_INT, what the copter is sent. ArduPlane only reads the altitude from it."""
    conn.mav.set_position_target_global_int_send(
        0, conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, 0b0000111111111000,
        int(lat * 1e7), int(lon * 1e7), alt, 0, 0, 0, 0, 0, 0, 0, 0)
    return "sent (no ACK exists for this message)"


# ---------------------------------------------------------------------------
# Flying to one target
# ---------------------------------------------------------------------------

def fly_to(conn, target, args, label):
    """Send the plane to `target` and watch. Returns True if it got within args.arrive metres."""
    send = goto_reposition if args.method == "reposition" else goto_target
    say(f"--- {label}: target ({target[0]:.5f}, {target[1]:.5f}) at {args.alt:.0f} m, method={args.method}")
    s = snapshot(conn)
    start_dist, best = distance_m(s["pos"], target), distance_m(s["pos"], target)
    say(f"    {start_dist:.0f} m away, bearing {bearing_deg(s['pos'], target):.0f}")
    result = send(conn, target[0], target[1], args.alt) if args.method == "target" else \
        send(conn, target[0], target[1], args.alt, args.radius)
    say(f"    plane's answer: {result}")
    start, last_print, last_send, accepted = time.time(), 0, time.time(), False
    while time.time() - start < args.leg_timeout:
        gcs_heartbeat(conn)
        s = snapshot(conn)
        if s is None:
            time.sleep(0.5)
            continue
        d = distance_m(s["pos"], target)
        best = min(best, d)
        if d < args.arrive:
            say(f"    REACHED, {d:.0f} m from the target after {time.time() - start:.0f} s")
            return True
        if args.method == "target" and time.time() - last_send >= 1:      # how a copter would be driven: repeat it
            last_send = time.time()
            goto_target(conn, target[0], target[1], args.alt)
        if time.time() - last_print >= 2:
            last_print = time.time()
            own = ""
            if s["tgt"] is not None:
                off = distance_m((s["tgt"].lat_int / 1e7, s["tgt"].lon_int / 1e7), target)
                accepted = accepted or off < 100
                own = f" | plane's own target is {off:.0f} m from ours"
            elif s["nav"] is not None:
                own = f" | autopilot says {s['nav'].wp_dist} m to its target"
            hdg = "?" if s["hdg"] is None else f"{s['hdg']:.0f}"
            say(f"    t={time.time() - start:3.0f}s mode={mode_name(s['mode'])} {d:5.0f} m to go, {s['gs']:.1f} m/s, "
                f"heading {hdg} (should be ~{bearing_deg(s['pos'], target):.0f}), alt {s['alt']:.0f} m{own}")
        # If the autopilot itself reports our target, it was accepted: do not second-guess a plane that is merely slow
        # (it spends ~10 s turning onto the line first). Without that read-back, 45 s with under 250 m gained is an
        # orbit, not a flight: orbiting alone swings it up to ~240 m nearer and farther, so less proves nothing.
        if not accepted and time.time() - start > 45 and best > start_dist - 250:
            say(f"    NOT MOVING TOWARD THE TARGET: closest it has been is {best:.0f} m, it started {start_dist:.0f} m away. "
                "The plane is circling; the target was not accepted.")
            return False
    say(f"    timed out after {args.leg_timeout:.0f} s, closest {best:.0f} m")
    return False


def main():
    ap = argparse.ArgumentParser(description="Fly the fixed-wing on its own to test that it obeys position commands")
    sim_config.add_argument(ap)
    ap.add_argument("--method", choices=["reposition", "target"], default="reposition",
                    help="reposition = MAV_CMD_DO_REPOSITION (correct for a plane); "
                         "target = SET_POSITION_TARGET_GLOBAL_INT (what the copter uses; the plane ignores lat/lon)")
    ap.add_argument("--distance", type=float, default=1200, help="metres from the start point to each waypoint")
    ap.add_argument("--legs", type=int, default=3, help="how many waypoints (a triangle around the start point)")
    ap.add_argument("--bearing", type=float, default=90, help="compass bearing of the first waypoint from the start point")
    ap.add_argument("--alt", type=float, default=80, help="altitude above home to fly the waypoints at, metres")
    ap.add_argument("--takeoff-alt", type=float, default=60, help="where TAKEOFF mode levels off (TKOFF_ALT in plane.parm)")
    ap.add_argument("--arrive", type=float, default=150, help="counts as arrived within this many metres (loiter radius is 120)")
    ap.add_argument("--radius", type=float, default=120, help="loiter radius for DO_REPOSITION, metres. ArduPlane only applies it "
                                                              "when > 0 (0 leaves the previous radius), so it is always sent")
    ap.add_argument("--leg-timeout", type=float, default=240, help="give up on a waypoint after this many seconds")
    ap.add_argument("--no-takeoff", action="store_true", help="the plane is already airborne")
    ap.add_argument("--no-rtl", action="store_true", help="do not switch to RTL at the end")
    args = ap.parse_args()
    sim_config.configure(args)
    if args.arrive < 130:
        say(f"--arrive {args.arrive:.0f} m is inside the plane's 120 m loiter radius, so it could never count as arrived: using 150 m")
        args.arrive = 150

    conn = connect()
    reached = []
    try:
        if not args.no_takeoff:
            start_up(conn, args.takeoff_alt)
        s = wait_for(conn, "a position", lambda s: True, timeout=30)
        if s is None:
            sys.exit("no position from the plane")
        if s["alt"] < 20:
            sys.exit(f"the plane is at {s['alt']:.0f} m: it is not flying. Run without --no-takeoff.")
        if s["mode"] != MODE_GUIDED:
            say("switching to GUIDED (the plane will circle where it is until it is given a target)")
            if not set_mode(conn, MODE_GUIDED):
                sys.exit("the plane would not enter GUIDED")
        home = s["pos"]
        waypoints = [offset(home, args.distance, args.bearing + i * 360 / args.legs) for i in range(args.legs)] \
            if args.legs > 1 else [offset(home, args.distance, args.bearing)]
        for i, wp in enumerate(waypoints, 1):
            reached.append(fly_to(conn, wp, args, f"waypoint {i}/{len(waypoints)}"))
            if not reached[-1]:
                break
    except KeyboardInterrupt:
        say("Ctrl+C")
    finally:
        if not args.no_rtl:
            say("switching to RTL")
            try:
                set_mode(conn, MODE_RTL)
            except Exception as e:
                say(f"could not switch to RTL: {e}")
    print()
    if reached and all(reached):
        say(f"RESULT: the plane flew to all {len(reached)} waypoints with method={args.method}.")
    else:
        say(f"RESULT: the plane did NOT reliably fly to its waypoints with method={args.method} "
            f"({sum(reached)} of {len(reached)} reached).")


if __name__ == "__main__":
    main()
