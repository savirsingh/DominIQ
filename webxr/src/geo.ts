/**
 * WGS84 lat/lon -> arctic-sim world metres.
 *
 * The sim's world axes are ArcticDEM grid axes (EPSG:3413, polar stereographic, true-scale
 * latitude 70 N, central meridian 45 W), centred on the site's bounds. Under `true_scale` world
 * metres are ground metres, so a grid offset is divided by the point scale factor `k`. This is the
 * same convention as `world_to_lonlat` / `grid_per_ground` in arctic-sim/terrain/make_world.py;
 * ignoring `k` would shift assets against the terrain by up to 19 m at the map edge.
 *
 * No imports, so it can be run and tested outside the bundler.
 */

export interface SiteMeta {
  name: string;
  location: { lat: number; lon: number };
  convergence_deg: number;
  grid: number;
  extent_m: number;
  scale_factor: number;
  true_scale: boolean;
  bounds_3413: { xmin: number; ymin: number; xmax: number; ymax: number };
  elevation_m: { min: number; max: number; range: number };
  heightmap_bits: number;
}

const A = 6378137.0;
const F = 1 / 298.257223563;
const E = Math.sqrt(F * (2 - F));
const LAT_TS = (70 * Math.PI) / 180;
const LON_0 = (-45 * Math.PI) / 180;

const t = (phi: number): number => {
  const s = Math.sin(phi);
  return Math.tan(Math.PI / 4 - phi / 2) / ((1 - E * s) / (1 + E * s)) ** (E / 2);
};
const m = (phi: number): number => Math.cos(phi) / Math.sqrt(1 - E * E * Math.sin(phi) ** 2);

/** EPSG:3413 grid metres (easting, northing). */
export function toEpsg3413(latDeg: number, lonDeg: number): { x: number; y: number } {
  const phi = (latDeg * Math.PI) / 180;
  const lambda = (lonDeg * Math.PI) / 180;
  const rho = (A * m(LAT_TS) * t(phi)) / t(LAT_TS);
  return { x: rho * Math.sin(lambda - LON_0), y: -rho * Math.cos(lambda - LON_0) };
}

export class Georef {
  private readonly cx: number;
  private readonly cy: number;
  private readonly k: number;

  constructor(readonly site: SiteMeta) {
    const b = site.bounds_3413;
    this.cx = (b.xmin + b.xmax) / 2;
    this.cy = (b.ymin + b.ymax) / 2;
    this.k = site.true_scale ? site.scale_factor : 1;
  }

  /** World metres: +x grid east, +y grid north, origin at the terrain centre. */
  toWorld(latDeg: number, lonDeg: number): { x: number; y: number } {
    const g = toEpsg3413(latDeg, lonDeg);
    return { x: (g.x - this.cx) / this.k, y: (g.y - this.cy) / this.k };
  }

  /** True if the world point lies on the terrain square. */
  contains(x: number, y: number): boolean {
    const half = this.site.extent_m / 2;
    return Math.abs(x) <= half && Math.abs(y) <= half;
  }
}
