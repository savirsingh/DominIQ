#!/usr/bin/env python3
"""Export an arctic-sim site into the files the WebXR map loads.

    python3 tools/prepare_terrain.py --site fort_ross
    python3 tools/prepare_terrain.py --site fort_ross --arctic-sim ../../arctic-sim

Reads  <arctic-sim>/out/<site>/{heightmap.png,albedo.png,terrain.json}
Writes public/terrain/<site>/{heightmap.png,albedo.jpg,site.json}

heightmap.png is copied as-is (8-bit, 2^n+1 square, row 0 = north). albedo.png is 4096^2 and
17 MB, so it is downscaled to a JPEG the Quest can decode quickly. site.json keeps only the
georeference the client needs; terrain.json also holds ~30 KB of course/water candidates.
"""
from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path

from PIL import Image

HERE = Path(__file__).resolve().parent.parent
DEFAULT_SIM = HERE.parent.parent / "arctic-sim"

KEEP = ("name", "location", "convergence_deg", "grid", "extent_m", "spacing_m", "scale_factor",
        "true_scale", "bounds_3413", "elevation_m", "heightmap_bits")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--site", default="fort_ross")
    ap.add_argument("--arctic-sim", type=Path, default=DEFAULT_SIM, help="arctic-sim checkout (default: %(default)s)")
    ap.add_argument("--albedo-size", type=int, default=2048, help="albedo edge in pixels (default: %(default)s)")
    args = ap.parse_args()

    src = args.arctic_sim / "out" / args.site
    if not (src / "terrain.json").exists():
        raise SystemExit(f"{src}/terrain.json not found; build the site first or pass --arctic-sim")
    dst = HERE / "public" / "terrain" / args.site
    dst.mkdir(parents=True, exist_ok=True)

    meta = json.loads((src / "terrain.json").read_text())
    site = {key: meta[key] for key in KEEP}
    if site["heightmap_bits"] != 8:
        raise SystemExit("only 8-bit heightmaps are supported")
    (dst / "site.json").write_text(json.dumps(site, indent=2) + "\n")

    shutil.copyfile(src / "heightmap.png", dst / "heightmap.png")

    albedo = Image.open(src / "albedo.png").convert("RGB")
    albedo = albedo.resize((args.albedo_size, args.albedo_size), Image.LANCZOS)
    albedo.save(dst / "albedo.jpg", quality=88, optimize=True)

    for f in sorted(dst.iterdir()):
        print(f"{f.relative_to(HERE)}  {f.stat().st_size / 1024:.0f} KB")


if __name__ == "__main__":
    main()
