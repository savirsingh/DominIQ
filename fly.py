#!/usr/bin/env python3
"""
Flight controller for Arctic SIM-8 assets.

Usage:
    python3 fly.py arm quadcopter
    python3 fly.py takeoff quadcopter 50
    python3 fly.py goto quadcopter 71.990 -94.830 80
    python3 fly.py status
    python3 fly.py land quadcopter

Add --sim local (or set SIM_TARGET=local) to use your own docker compose sim instead of the
shared one over WireGuard, e.g. python3 fly.py --sim local status
"""

from pymavlink import mavutil
import argparse
import sim_config
import time
import math
import sys

ASSETS = sim_config.ASSETS


def connect(name):
    conn = mavutil.mavlink_connection(sim_config.mavlink_url(name), source_system=255)
    for _ in range(5):
        conn.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        time.sleep(0.2)
    conn.mav.request_data_stream_send(0, 0, mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
    msg = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
    if not msg:
        print(f"No heartbeat from {name}")
        sys.exit(1)
    return conn


def set_guided_mode(conn):
    conn.mav.set_mode_send(
        conn.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        4  # GUIDED mode for ArduCopter
    )
    time.sleep(1)
    print("  Mode set to GUIDED")


def arm(conn, name):
    print(f"Arming {name}...")
    set_guided_mode(conn)
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0)
    msg = conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
    if msg and msg.result == 0:
        print(f"  {name} armed!")
    else:
        print(f"  Arm result: {msg}")


def takeoff(conn, name, alt):
    print(f"Taking off {name} to {alt}m...")
    arm(conn, name)
    time.sleep(1)
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, alt)
    print(f"  Takeoff command sent. Climbing to {alt}m...")

    start = time.time()
    while time.time() - start < 30:
        conn.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        gps = conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
        if gps:
            cur_alt = gps.relative_alt / 1e3
            print(f"  Altitude: {cur_alt:.1f}m / {alt}m", end="\r")
            if cur_alt >= alt * 0.9:
                print(f"\n  Reached target altitude!")
                return
        time.sleep(1)
    print(f"\n  Timeout waiting for altitude (may still be climbing)")


def goto(conn, name, lat, lon, alt):
    print(f"Sending {name} to ({lat}, {lon}) at {alt}m...")
    conn.mav.set_position_target_global_int_send(
        0, conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,  # position only
        int(lat * 1e7), int(lon * 1e7), alt,
        0, 0, 0,
        0, 0, 0,
        0, 0)
    print(f"  Waypoint sent. Heading to target...")

    start = time.time()
    while time.time() - start < 60:
        conn.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        gps = conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
        if gps:
            cur_lat = gps.lat / 1e7
            cur_lon = gps.lon / 1e7
            cur_alt = gps.relative_alt / 1e3
            dlat = (lat - cur_lat) * 111320
            dlon = (lon - cur_lon) * 111320 * math.cos(math.radians(lat))
            dist = math.sqrt(dlat**2 + dlon**2)
            print(f"  pos=({cur_lat:.6f},{cur_lon:.6f}) alt={cur_alt:.1f}m dist={dist:.0f}m", end="\r")
            if dist < 5:
                print(f"\n  Arrived at target!")
                return
        time.sleep(1)
    print(f"\n  Timeout (may still be en route)")


def land(conn, name):
    print(f"Landing {name}...")
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0)
    print("  Land command sent.")


def status_all():
    print("=" * 60)
    print("ASSET STATUS")
    print("=" * 60)
    for name, info in ASSETS.items():
        try:
            conn = connect(name)
            gps = conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=3)
            att = conn.recv_match(type="ATTITUDE", blocking=True, timeout=2)
            hb = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
            if gps:
                lat = gps.lat / 1e7
                lon = gps.lon / 1e7
                alt = gps.relative_alt / 1e3
                armed = "ARMED" if (hb and hb.base_mode & 128) else "disarmed"
                print(f"  {name:15s}  ({lat:.6f}, {lon:.6f})  alt={alt:6.1f}m  {armed}")
            else:
                print(f"  {name:15s}  no GPS")
            conn.close()
        except Exception as e:
            print(f"  {name:15s}  offline ({e})")


def usage():
    print(__doc__)
    sys.exit(1)


if __name__ == "__main__":
    sim_parser = argparse.ArgumentParser(add_help=False)
    sim_config.add_argument(sim_parser)
    sim_args, sys.argv[1:] = sim_parser.parse_known_args()
    sim_config.configure(sim_args)

    if len(sys.argv) < 2:
        usage()

    cmd = sys.argv[1]

    if cmd == "status":
        status_all()
    elif cmd == "arm" and len(sys.argv) == 3:
        conn = connect(sys.argv[2])
        arm(conn, sys.argv[2])
        conn.close()
    elif cmd == "takeoff" and len(sys.argv) == 4:
        conn = connect(sys.argv[2])
        takeoff(conn, sys.argv[2], float(sys.argv[3]))
        conn.close()
    elif cmd == "goto" and len(sys.argv) == 6:
        conn = connect(sys.argv[2])
        goto(conn, sys.argv[2], float(sys.argv[3]), float(sys.argv[4]), float(sys.argv[5]))
        conn.close()
    elif cmd == "land" and len(sys.argv) == 3:
        conn = connect(sys.argv[2])
        land(conn, sys.argv[2])
        conn.close()
    else:
        usage()
