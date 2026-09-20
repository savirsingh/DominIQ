import {
  BufferGeometry,
  DirectionalLight,
  Float32BufferAttribute,
  Group,
  HemisphereLight,
  Mesh,
  MeshLambertMaterial,
  SRGBColorSpace,
  TextureLoader,
  Uint32BufferAttribute,
  Vector3,
} from '@iwsdk/core';
import { Georef, type SiteMeta } from './geo';

export interface TerrainOptions {
  /** Edge of the square map on the table, in meters. */
  widthM?: number;
  /** Vertical exaggeration. Real relief is 252 m over 6.5 km, which is nearly flat at tabletop scale. */
  exaggeration?: number;
  /** Heightmap pixels per mesh cell. 4 turns the 1025 px grid into a 257 x 257 mesh. */
  stride?: number;
}

export interface TerrainMap {
  /** Add this to the placed map marker. Origin is the map centre at sea level, +y up, north is -z. */
  readonly group: Group;
  readonly georef: Georef;
  /** Edge of the map, in meters. Same on both axes: the terrain is square. */
  readonly widthM: number;
  /** Local map position (map meters) for a lat/lon and an AMSL altitude in meters. */
  localPosition(lat: number, lon: number, altM: number, out: Vector3): Vector3;
  /** Local height of the terrain surface under a local x/z (map meters). */
  groundHeight(x: number, z: number): number;
}

async function readHeights(url: string): Promise<{ data: Float32Array; size: number }> {
  const blob = await (await fetch(url)).blob();
  // 'none': the bytes ARE the elevation, so no colour-space conversion may touch them.
  const bitmap = await createImageBitmap(blob, { colorSpaceConversion: 'none' });
  const canvas = new OffscreenCanvas(bitmap.width, bitmap.height);
  const ctx = canvas.getContext('2d', { willReadFrequently: true });
  if (!ctx) throw new Error('No 2D canvas available to read the heightmap.');
  ctx.drawImage(bitmap, 0, 0);
  const rgba = ctx.getImageData(0, 0, bitmap.width, bitmap.height).data;
  const data = new Float32Array(bitmap.width * bitmap.height);
  for (let i = 0; i < data.length; i++) data[i] = rgba[i * 4] / 255;
  return { data, size: bitmap.width };
}

/** Build the terrain mesh from a site exported by tools/prepare_terrain.py. */
export async function loadTerrain(baseUrl: string, options: TerrainOptions = {}): Promise<TerrainMap> {
  const { widthM = 0.9, exaggeration = 3, stride = 4 } = options;
  const base = baseUrl.replace(/\/+$/u, '');

  const site = (await (await fetch(`${base}/site.json`)).json()) as SiteMeta;
  const { data: heights, size } = await readHeights(`${base}/heightmap.png`);
  if (size !== site.grid) throw new Error(`heightmap is ${size} px but site.json says ${site.grid}`);

  const georef = new Georef(site);
  const metresToMap = widthM / site.extent_m;
  const heightScale = site.elevation_m.range * metresToMap * exaggeration;

  // Row 0 of the image is north (checked against the sim's own tower altitudes), and north is -z.
  const cells = Math.floor((size - 1) / stride);
  const verts = cells + 1;
  const positions = new Float32Array(verts * verts * 3);
  const uvs = new Float32Array(verts * verts * 2);
  for (let r = 0; r < verts; r++) {
    for (let c = 0; c < verts; c++) {
      const i = r * verts + c;
      const h = heights[Math.min(r * stride, size - 1) * size + Math.min(c * stride, size - 1)];
      positions[i * 3] = (c / cells - 0.5) * widthM;
      positions[i * 3 + 1] = h * heightScale;
      positions[i * 3 + 2] = (r / cells - 0.5) * widthM;
      uvs[i * 2] = c / cells;
      uvs[i * 2 + 1] = 1 - r / cells;
    }
  }
  const indices = new Uint32Array(cells * cells * 6);
  let n = 0;
  for (let r = 0; r < cells; r++) {
    for (let c = 0; c < cells; c++) {
      const a = r * verts + c;
      const b = a + 1;
      const d = a + verts;
      const e = d + 1;
      indices.set([a, d, b, b, d, e], n);
      n += 6;
    }
  }
  const geometry = new BufferGeometry();
  geometry.setAttribute('position', new Float32BufferAttribute(positions, 3));
  geometry.setAttribute('uv', new Float32BufferAttribute(uvs, 2));
  geometry.setIndex(new Uint32BufferAttribute(indices, 1));
  geometry.computeVertexNormals();

  const map = await new TextureLoader().loadAsync(`${base}/albedo.jpg`);
  map.colorSpace = SRGBColorSpace;
  map.anisotropy = 4;

  const group = new Group();
  group.add(new Mesh(geometry, new MeshLambertMaterial({ map })));
  // The scene has no lights of its own. Without these the relief is invisible.
  group.add(new HemisphereLight(0xffffff, 0x666666, 1.6));
  const sun = new DirectionalLight(0xffffff, 1.4);
  sun.position.set(-0.5, 1, 0.4);
  group.add(sun);

  const groundHeight = (x: number, z: number): number => {
    // Bilinear sample of the heightmap at a local x/z.
    const px = Math.min(Math.max((x / widthM + 0.5) * (size - 1), 0), size - 1);
    const py = Math.min(Math.max((z / widthM + 0.5) * (size - 1), 0), size - 1);
    const x0 = Math.min(Math.floor(px), size - 2);
    const y0 = Math.min(Math.floor(py), size - 2);
    const fx = px - x0;
    const fy = py - y0;
    const at = (row: number, col: number): number => heights[row * size + col];
    const top = at(y0, x0) * (1 - fx) + at(y0, x0 + 1) * fx;
    const bottom = at(y0 + 1, x0) * (1 - fx) + at(y0 + 1, x0 + 1) * fx;
    return (top * (1 - fy) + bottom * fy) * heightScale;
  };

  return {
    group,
    georef,
    widthM,
    localPosition(lat, lon, altM, out) {
      const w = georef.toWorld(lat, lon);
      return out.set(w.x * metresToMap, altM * metresToMap * exaggeration, -w.y * metresToMap);
    },
    groundHeight,
  };
}
