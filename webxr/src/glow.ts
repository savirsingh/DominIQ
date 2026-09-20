import {
  DataTexture,
  DoubleSide,
  Group,
  MathUtils,
  Mesh,
  MeshBasicMaterial,
  RGBAFormat,
  RingGeometry,
  Sprite,
  SpriteMaterial,
  AdditiveBlending,
  LinearFilter,
} from '@iwsdk/core';

const RING_COUNT = 3;
const PERIOD_S = 2.4; // one ring's trip from the pin to full size
const R_MIN = 0.012; // meters on the map, at birth
const R_MAX = 0.1;
const LIFT = 0.004; // keeps the rings off the water surface they lie on (avoids z-fighting)

/** A soft white disc that fades to nothing at its edge. Tinted by the sprite's color. */
function haloTexture(): DataTexture {
  const size = 64;
  const data = new Uint8Array(size * size * 4);
  for (let y = 0; y < size; y++) {
    for (let x = 0; x < size; x++) {
      const r = Math.hypot((x + 0.5) / size - 0.5, (y + 0.5) / size - 0.5) * 2; // 0 centre, 1 edge
      const a = Math.max(0, 1 - r) ** 2;
      data.set([255, 255, 255, Math.round(a * 255)], (y * size + x) * 4);
    }
  }
  const texture = new DataTexture(data, size, size, RGBAFormat);
  texture.magFilter = LinearFilter;
  texture.minFilter = LinearFilter;
  texture.needsUpdate = true;
  return texture;
}

/**
 * Rings that ripple outward from a point, like a sonar ping, over a soft halo. Lies flat, so it
 * reads as a ping on the water when the map is seen from above. Call update() every frame.
 */
export class PulseRings extends Group {
  /** 0..1, follows the ring cycle. The pin can scale its body with it so it breathes in time. */
  pulse = 0;
  private readonly rings: Mesh[] = [];
  private readonly halo: Sprite;
  private time = 0;

  constructor(color: number) {
    super();
    const geometry = new RingGeometry(0.86, 1, 64).rotateX(-Math.PI / 2);
    for (let i = 0; i < RING_COUNT; i++) {
      const ring = new Mesh(
        geometry,
        new MeshBasicMaterial({ color, transparent: true, depthWrite: false, side: DoubleSide }),
      );
      ring.position.y = LIFT;
      this.rings.push(ring);
      this.add(ring);
    }
    this.halo = new Sprite(
      new SpriteMaterial({
        map: haloTexture(),
        color,
        transparent: true,
        depthWrite: false,
        blending: AdditiveBlending,
      }),
    );
    this.add(this.halo);
    this.update(0);
  }

  update(deltaSeconds: number): void {
    this.time += deltaSeconds;
    const cycle = this.time / PERIOD_S;
    this.rings.forEach((ring, i) => {
      const phase = (cycle + i / RING_COUNT) % 1;
      // Expands fast then slows; fades in over the first few percent so a ring never pops into view.
      ring.scale.setScalar(MathUtils.lerp(R_MIN, R_MAX, 1 - (1 - phase) ** 2));
      (ring.material as MeshBasicMaterial).opacity = 0.9 * (1 - phase) ** 1.5 * Math.min(1, phase / 0.06);
    });
    this.pulse = 0.5 + 0.5 * Math.sin(cycle * Math.PI * 2);
    this.halo.scale.setScalar(0.05 + 0.03 * this.pulse);
    (this.halo.material as SpriteMaterial).opacity = 0.55 + 0.3 * this.pulse;
  }
}
