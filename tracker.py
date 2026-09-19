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
"""

import math
import argparse
import time
import json

# Earth radius in metres (WGS-84 mean)
EARTH_RADIUS = 6_371_000

# SIM asset connection info (UDP ports — SITL only streams over UDP)
ASSETS = {
    "quadcopter": {"host": "10.99.7.1", "udp": 14550, "type": "copter"},
    "fixed-wing": {"host": "10.99.7.1", "udp": 14560, "type": "plane"},
    "tower-1":    {"host": "10.99.7.1", "udp": 14580, "type": "tower"},
    "tower-2":    {"host": "10.99.7.1", "udp": 14590, "type": "tower"},
}


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
    for name, info in ASSETS.items():
        addr = f"udpout:{info['host']}:{info['udp']}"
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
    args = parser.parse_args()

    if args.live:
        run_live()
    else:
        run_test()
