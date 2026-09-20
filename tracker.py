#!/usr/bin/env python3
"""
Boat position estimator for Arctic SIM-8.

Given a drone's GPS (lat, lon, altitude) and the camera's look-down angle
+ heading, computes the estimated lat/lon of the target boat using trig:

    horizontal_distance = altitude / tan(elevation_angle)

where elevation_angle is measured from horizontal (90 = straight down, 0 = horizon).

The horizontal distance is projected along the camera heading to get
a lat/lon offset from the drone.

Usage:
    python3 tracker.py              # run with test coordinates
    python3 tracker.py --live       # connect to SIM MAVLink (when available)
    python3 tracker.py --live --sim local   # your own docker compose sim (or SIM_TARGET=local)
"""

import math
import argparse
import time
import json

import sim_config

# Earth radius in metres (WGS-84 mean)
EARTH_RADIUS = 6_371_000

# SIM asset connection info (UDP ports — SITL only streams over UDP)
ASSETS = sim_config.ASSETS


def estimate_boat_position(drone_lat, drone_lon, drone_alt,
                           camera_elevation_deg, camera_heading_deg):
    """
    Estimate the boat's GPS position from a drone observation.

    Args:
        drone_lat:            Drone latitude (degrees)
        drone_lon:            Drone longitude (degrees)
        drone_alt:            Drone altitude AGL (metres)
        camera_elevation_deg: Camera angle from horizontal toward the boat.
                              90 = looking straight down, 0 = looking at horizon.
        camera_heading_deg:   Compass heading the camera is pointing (0=N, 90=E, etc.)

    Returns:
        (boat_lat, boat_lon, horizontal_distance_m)
    """
    elev_rad = math.radians(camera_elevation_deg)
    if elev_rad <= 0 or elev_rad > math.pi / 2:
        raise ValueError(f"Elevation must be between 0 (exclusive) and 90 (inclusive), got {camera_elevation_deg}")

    horizontal_dist = drone_alt / math.tan(elev_rad)

    heading_rad = math.radians(camera_heading_deg)
    boat_lat, boat_lon = offset_lat_lon(
        drone_lat, drone_lon, horizontal_dist, heading_rad
    )

    return boat_lat, boat_lon, horizontal_dist


def _rx(v, a):
    c, s = math.cos(a), math.sin(a)
    return (v[0], c * v[1] - s * v[2], s * v[1] + c * v[2])


def _ry(v, a):
    c, s = math.cos(a), math.sin(a)
    return (c * v[0] + s * v[2], v[1], -s * v[0] + c * v[2])


def _rz(v, a):
    c, s = math.cos(a), math.sin(a)
    return (c * v[0] - s * v[1], s * v[0] + c * v[1], v[2])


def estimate_from_pixel(drone_lat, drone_lon, height_m, yaw_deg, pitch_deg, roll_deg,
                        px, py, img_w, img_h, hfov_rad, tilt_down_deg):
    """
    Estimate a target's GPS position from where it appears in a drone frame.

    Casts the pixel's ray from the camera through the drone's attitude and intersects
    it with a flat water plane. The camera is rigidly mounted looking forward, tilted
    tilt_down_deg below the airframe's nose axis, with no roll about its optical axis.

    Args:
        height_m:   drone height above the water plane (AMSL altitude in the sim)
        yaw_deg/pitch_deg/roll_deg: ArduPilot ATTITUDE (yaw clockwise from north,
                    pitch + = nose up, roll + = right wing down)
        px, py:     target pixel (origin top-left)
        hfov_rad:   camera horizontal field of view
        tilt_down_deg: camera downtilt relative to the airframe nose axis

    Returns:
        (lat, lon, ground_distance_m)

    Raises ValueError if the ray does not reach the water (pixel at/above the horizon).
    """
    f = (img_w / 2) / math.tan(hfov_rad / 2)
    # Body frame is forward / right / down.
    ray = (1.0, (px - img_w / 2) / f, (py - img_h / 2) / f)
    ray = _ry(ray, -math.radians(tilt_down_deg))
    ray = _rx(ray, math.radians(roll_deg))
    ray = _ry(ray, math.radians(pitch_deg))
    north, east, down = _rz(ray, math.radians(yaw_deg))
    if down <= 1e-6 or height_m <= 0:
        raise ValueError("Ray does not reach the water")

    t = height_m / down
    dist = t * math.hypot(north, east)
    lat, lon = offset_lat_lon(drone_lat, drone_lon, dist, math.atan2(east, north))
    return lat, lon, dist


def offset_lat_lon(lat, lon, distance_m, bearing_rad):
    """
    Compute a new lat/lon given a start point, distance, and bearing.
    Uses the spherical-Earth forward-azimuth formula.
    """
    lat_r = math.radians(lat)
    lon_r = math.radians(lon)
    d_over_r = distance_m / EARTH_RADIUS

    new_lat = math.asin(
        math.sin(lat_r) * math.cos(d_over_r)
        + math.cos(lat_r) * math.sin(d_over_r) * math.cos(bearing_rad)
    )
    new_lon = lon_r + math.atan2(
        math.sin(bearing_rad) * math.sin(d_over_r) * math.cos(lat_r),
        math.cos(d_over_r) - math.sin(lat_r) * math.sin(new_lat),
    )

    return math.degrees(new_lat), math.degrees(new_lon)


def haversine(lat1, lon1, lat2, lon2):
    """Distance in metres between two lat/lon points."""
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return EARTH_RADIUS * 2 * math.asin(math.sqrt(a))


def run_test():
    """Verify the estimation math with a round-trip test."""
    drone_lat, drone_lon, drone_alt = 71.9958, -94.8393, 100.0
    camera_elevation, camera_heading = 45.0, 180.0

    est_lat, est_lon, est_dist = estimate_boat_position(
        drone_lat, drone_lon, drone_alt, camera_elevation, camera_heading)

    print("=" * 60)
    print("BOAT POSITION ESTIMATOR — MATH SELF-TEST")
    print("=" * 60)
    print(f"  Drone:      ({drone_lat}, {drone_lon}) alt={drone_alt}m")
    print(f"  Camera:     elev={camera_elevation}° hdg={camera_heading}°")
    print(f"  Estimated:  ({est_lat:.6f}, {est_lon:.6f})")
    print(f"  Horiz dist: {est_dist:.1f}m")

    expected_dist = drone_alt / math.tan(math.radians(camera_elevation))
    actual_dist = haversine(drone_lat, drone_lon, est_lat, est_lon)
    error = abs(actual_dist - expected_dist)
    print(f"  Round-trip error: {error:.4f}m")
    print(f"  Result: {'PASS' if error < 1.0 else 'FAIL'}")
    print()


def run_live():
    """Connect to SIM drones via MAVLink and estimate boat position from live telemetry."""
    try:
        from pymavlink import mavutil
    except ImportError:
        print("ERROR: pymavlink not installed. Run: pip install pymavlink")
        return

    print("=" * 60)
    print("BOAT POSITION ESTIMATOR — LIVE MODE")
    print("=" * 60)
    print()

    connections = {}
    for name in ASSETS:
        addr = sim_config.mavlink_url(name)
        print(f"Connecting to {name} at {addr}...")
        try:
            conn = mavutil.mavlink_connection(addr, source_system=255)
            for _ in range(5):
                conn.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0,
                )
                time.sleep(0.2)

            conn.mav.request_data_stream_send(
                0, 0,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                4, 1,
            )

            msg = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
            if msg:
                print(f"  Connected (type={msg.type})")
                connections[name] = conn
            else:
                print(f"  No heartbeat — SITL may not be running")
                conn.close()
        except Exception as e:
            print(f"  Failed: {e}")

    if not connections:
        print("\nNo active MAVLink connections. The SIM assets may need a reset.")
        print("Use the Reset button in the SIM UI, or try again later.")
        print("Running test mode instead...\n")
        return run_test()

    print(f"\n{len(connections)} active connection(s). Polling telemetry...\n")
    print("(Press Ctrl+C to stop)\n")

    try:
        while True:
            for name, conn in connections.items():
                conn.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0,
                )

                gps = conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
                att = conn.recv_match(type="ATTITUDE", blocking=True, timeout=1)

                if gps:
                    lat = gps.lat / 1e7
                    lon = gps.lon / 1e7
                    alt = gps.relative_alt / 1e3
                    hdg = gps.hdg / 100.0

                    pitch_deg = math.degrees(att.pitch) if att else 0
                    yaw_deg = math.degrees(att.yaw) if att else hdg

                    camera_elevation = max(0.1, min(90, 90 + pitch_deg))
                    camera_heading = yaw_deg % 360

                    if alt < 1.0:
                        print(f"[{name}] pos=({lat:.6f},{lon:.6f}) alt={alt:.1f}m — on ground, no estimate")
                    else:
                        est_lat, est_lon, est_dist = estimate_boat_position(
                            lat, lon, alt, camera_elevation, camera_heading
                        )
                        print(
                            f"[{name}] pos=({lat:.6f},{lon:.6f}) alt={alt:.1f}m "
                            f"elev={camera_elevation:.1f}° hdg={camera_heading:.1f}° "
                            f"→ boat≈({est_lat:.6f},{est_lon:.6f}) dist={est_dist:.1f}m"
                        )
                else:
                    print(f"[{name}] no GPS data")

            print()
            time.sleep(2)

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        for conn in connections.values():
            conn.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Boat position estimator for Arctic SIM-8")
    parser.add_argument("--live", action="store_true", help="Connect to SIM MAVLink")
    sim_config.add_argument(parser)
    args = parser.parse_args()
    sim_config.configure(args)

    if args.live:
        run_live()
    else:
        run_test()
