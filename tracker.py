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
    """Run with test coordinates to verify the math."""

    # --- Test scenario ---
    # Boat is actually at this position (ground truth for validation)
    actual_boat_lat = 71.990
    actual_boat_lon = -94.830

    # Drone (quadcopter) is hovering nearby
    drone_lat = 71.9958
    drone_lon = -94.8393
    drone_alt = 100.0  # 100m AGL

    # Compute bearing and elevation from drone to boat (as if the camera found it)
    dist_to_boat = haversine(drone_lat, drone_lon, actual_boat_lat, actual_boat_lon)
    camera_elevation = math.degrees(math.atan2(drone_alt, dist_to_boat))

    # Bearing from drone to boat
    lat1, lon1 = math.radians(drone_lat), math.radians(drone_lon)
    lat2, lon2 = math.radians(actual_boat_lat), math.radians(actual_boat_lon)
    dlon = lon2 - lon1
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    camera_heading = (math.degrees(math.atan2(x, y)) + 360) % 360

    print("=" * 60)
    print("BOAT POSITION ESTIMATOR — TEST MODE")
    print("=" * 60)

    print(f"\n--- Ground truth ---")
    print(f"  Actual boat position:  {actual_boat_lat:.6f}, {actual_boat_lon:.6f}")

    print(f"\n--- Drone telemetry ---")
    print(f"  Drone position:        {drone_lat:.6f}, {drone_lon:.6f}")
    print(f"  Drone altitude (AGL):  {drone_alt:.1f} m")
    print(f"  True distance to boat: {dist_to_boat:.1f} m")
    print(f"  Camera elevation:      {camera_elevation:.2f} deg (from horizontal)")
    print(f"  Camera heading:        {camera_heading:.2f} deg")

    # Now estimate the boat position (this is what the real system does)
    est_lat, est_lon, est_dist = estimate_boat_position(
        drone_lat, drone_lon, drone_alt,
        camera_elevation, camera_heading
    )

    error = haversine(actual_boat_lat, actual_boat_lon, est_lat, est_lon)

    print(f"\n--- Estimated boat position ---")
    print(f"  Estimated position:    {est_lat:.6f}, {est_lon:.6f}")
    print(f"  Estimated distance:    {est_dist:.1f} m")
    print(f"  Position error:        {error:.2f} m")
    print(f"  Result:                {'PASS' if error < 1.0 else 'FAIL'} (< 1m tolerance)")

    # --- Multi-drone triangulation test ---
    print(f"\n{'=' * 60}")
    print("MULTI-DRONE TRIANGULATION TEST")
    print("=" * 60)

    observations = [
        {
            "name": "quadcopter",
            "lat": 71.9958, "lon": -94.8393, "alt": 100.0,
        },
        {
            "name": "tower-1",
            "lat": 71.9807, "lon": -94.8537, "alt": 15.0,
        },
        {
            "name": "tower-2",
            "lat": 72.0118, "lon": -94.8047, "alt": 15.0,
        },
    ]

    estimates = []
    for obs in observations:
        dist = haversine(obs["lat"], obs["lon"], actual_boat_lat, actual_boat_lon)
        elev = math.degrees(math.atan2(obs["alt"], dist))

        lat1, lon1 = math.radians(obs["lat"]), math.radians(obs["lon"])
        lat2, lon2 = math.radians(actual_boat_lat), math.radians(actual_boat_lon)
        dlon = lon2 - lon1
        x = math.sin(dlon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        heading = (math.degrees(math.atan2(x, y)) + 360) % 360

        e_lat, e_lon, e_dist = estimate_boat_position(
            obs["lat"], obs["lon"], obs["alt"], elev, heading
        )
        error = haversine(actual_boat_lat, actual_boat_lon, e_lat, e_lon)
        estimates.append({"name": obs["name"], "lat": e_lat, "lon": e_lon, "error": error})
        print(f"\n  {obs['name']}:")
        print(f"    Observer at:      {obs['lat']:.6f}, {obs['lon']:.6f}, alt={obs['alt']}m")
        print(f"    Distance to boat: {dist:.1f} m")
        print(f"    Elevation angle:  {elev:.2f} deg")
        print(f"    Heading:          {heading:.2f} deg")
        print(f"    Estimated boat:   {e_lat:.6f}, {e_lon:.6f}")
        print(f"    Error:            {error:.2f} m")

    # Average the estimates (simple fusion)
    avg_lat = sum(e["lat"] for e in estimates) / len(estimates)
    avg_lon = sum(e["lon"] for e in estimates) / len(estimates)
    avg_error = haversine(actual_boat_lat, actual_boat_lon, avg_lat, avg_lon)

    print(f"\n--- Fused estimate (average of {len(estimates)} observers) ---")
    print(f"  Fused position:  {avg_lat:.6f}, {avg_lon:.6f}")
    print(f"  Fused error:     {avg_error:.2f} m")
    print(f"  Result:          {'PASS' if avg_error < 1.0 else 'FAIL'}")
    print()

    return {
        "actual": {"lat": actual_boat_lat, "lon": actual_boat_lon},
        "estimates": estimates,
        "fused": {"lat": avg_lat, "lon": avg_lon, "error_m": avg_error},
    }


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
