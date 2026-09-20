"""Turn the mission's live state into text a language model can answer from.

Language models are unreliable at lat/lon arithmetic, so distances and bearings between assets and the
boat are computed here and stated outright.
"""
from __future__ import annotations

import math
import os
from typing import Optional

EARTH_RADIUS_M = 6371000.0
STALE_S = 5.0  # an asset silent this long is flagged as possibly stale

_COMPASS = ["north", "north-northeast", "northeast", "east-northeast", "east", "east-southeast", "southeast",
            "south-southeast", "south", "south-southwest", "southwest", "west-southwest", "west",
            "west-northwest", "northwest", "north-northwest"]
_KIND = {"copter": "quadcopter", "plane": "fixed-wing aircraft", "tower": "camera tower", "boat": "target vessel"}


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    p1, p2 = math.radians(lat1), math.radians(lat2)
    a = math.sin((p2 - p1) / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(math.radians(lon2 - lon1) / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(a))


def bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Initial true bearing from point 1 to point 2, degrees clockwise from north."""
    p1, p2, dl = math.radians(lat1), math.radians(lat2), math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return math.degrees(math.atan2(y, x)) % 360


def compass(bearing: float) -> str:
    return _COMPASS[int((bearing + 11.25) // 22.5) % 16]


def fmt_distance(metres: float) -> str:
    return f"{metres:.0f} m" if round(metres) < 1000 else f"{metres / 1000:.1f} km"   # 999.6 m is "1.0 km", not "1000 m"


def describe_offset(a: dict, b: dict) -> str:
    """Where `b` is relative to `a`, in words."""
    d = haversine_m(a["lat"], a["lon"], b["lat"], b["lon"])
    brg = bearing_deg(a["lat"], a["lon"], b["lat"], b["lon"])
    return f"{fmt_distance(d)} to the {compass(brg)} ({brg:.0f} degrees)"


def read_log_state(path: str, max_lines: int = 60) -> Optional[dict]:
    """The mission log's recent status, detections, motion and errors (same idea as voice.py). None if there's no log."""
    try:
        with open(path, "rb") as f:
            f.seek(0, os.SEEK_END)
            f.seek(max(0, f.tell() - 64 * 1024))
            lines = f.read().decode("utf-8", "replace").splitlines()[-max_lines:]
    except OSError:
        return None

    assets, boat, in_status = [], "", False
    for line in lines:
        if "--- STATUS ---" in line:
            in_status, assets, boat = True, [], ""
        elif in_status:
            if line.strip() == "":
                in_status = False
            elif "BOAT(" in line:
                boat = line.strip()
            elif any(n in line for n in ("quadcopter", "fixed-wing", "tower")):
                assets.append(line.strip())
    return {
        "assets": assets,
        "boat": boat,
        "detections": [l for l in lines if "BOAT DETECTED" in l or "BOAT SPOTTED" in l][-3:],
        "motion": [l for l in lines if "MOTION DETECTED" in l or "detected motion" in l][-3:],
        "errors": [l for l in lines if "ARM FAILED" in l or "error:" in l.lower()][-3:],
    }


def build_context(snapshot: Optional[dict], focus: Optional[dict] = None, log_state: Optional[dict] = None) -> str:
    """The live-state section of the system prompt."""
    lines = []
    assets = list((snapshot or {}).get("assets") or [])
    if snapshot is None:
        lines.append("Live position feed: unavailable, so there is no position data right now.")
    elif not assets:
        lines.append("Live position feed: connected, but no asset has reported a position yet (they may still be taking off).")
    else:
        lines.append("Live asset positions (from the mission feed):")
        for a in assets:
            if a.get("kind") == "boat":
                continue
            stale = f" (no update for {a['age']:.0f} s, may be stale)" if a.get("age", 0) > STALE_S else ""
            lines.append(f"  {a['name']} ({_KIND.get(a.get('kind'), a.get('kind'))}): latitude {a['lat']:.5f}, "
                         f"longitude {a['lon']:.5f}, altitude {a['alt']:.0f} m above sea level{stale}")

    boat = next((a for a in assets if a.get("kind") == "boat"), None)
    others = [a for a in assets if a.get("kind") != "boat"]
    if boat:
        seen = f" It was first detected {boat['first_seen_age']:.0f} s ago." if boat.get("first_seen_age") is not None else ""
        lines.append(f"Boat (target vessel): the mission's estimated position is latitude {boat['lat']:.5f}, "
                     f"longitude {boat['lon']:.5f}.{seen} This is an estimate from camera detections, not ground truth.")
        for a in others:
            lines.append(f"  From {a['name']}, the boat is {describe_offset(a, boat)}.")
    elif snapshot is not None:
        lines.append("Boat: not detected yet.")

    if len(others) > 1:
        lines.append("Distances between assets:")
        for i, a in enumerate(others):
            for b in others[i + 1:]:
                lines.append(f"  {a['name']} to {b['name']}: {describe_offset(a, b)}.")

    if focus:
        if focus.get("hovered"):
            lines.append(f"The operator is pointing at {focus['hovered']} right now, so 'this' or 'that' means it.")
        if focus.get("locked"):
            lines.append(f"The operator has pinned these tags open: {', '.join(focus['locked'])}.")

    if log_state:
        if log_state["assets"]:
            lines.append("Latest mission status lines:")
            lines.extend(f"  {a}" for a in log_state["assets"])
        for key, title in (("boat", "Mission boat line"), ("detections", "Recent boat detections"),
                           ("motion", "Recent tower motion events"), ("errors", "Recent errors")):
            value = log_state[key]
            if value:
                lines.append(f"{title}:" if isinstance(value, list) else f"{title}: {value}")
                if isinstance(value, list):
                    lines.extend(f"  {v}" for v in value)
    return "\n".join(lines)


def build_system_prompt(context: str) -> str:
    return "\n".join([
        "You are the AI operator assistant for the Arctic SIM-8 drone mission.",
        "The operator is wearing a Meta Quest headset, looking at a tabletop map of the terrain with live pins",
        "for the quadcopter, the fixed-wing aircraft, two camera towers and the target boat.",
        "Answer the operator's questions about what is happening. This is a voice interface: keep answers to",
        "2-3 sentences, in natural spoken language, with no markdown and no lists. Say distances in metres or",
        "kilometres and directions as compass words. If you do not know something from the data below, say so.",
        "",
        "=== CURRENT MISSION STATE ===",
        context,
    ])
