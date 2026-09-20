import { CanvasTexture, Group, Mesh, MeshBasicMaterial, PlaneGeometry, SRGBColorSpace } from '@iwsdk/core';
import { roundedRect } from './canvas-util';
import type { FeedAsset } from './feed';

const FADE_IN_S = 0.3;
const HOLD_S = 2.4;
const FADE_OUT_S = 0.6;
const DURATION_S = FADE_IN_S + HOLD_S + FADE_OUT_S;
const RISE_M = 0.02; // it drifts up this far into place while fading in
const FRESH_S = 30; // a boat first seen longer ago than this is old news: don't alert for it

// A small toast, about the size of an info tag, in the upper part of the view (clear of the map on the table).
const WIDTH_M = 0.36;
const HEIGHT_M = 0.075;
const DISTANCE_M = 1.0;
const HEIGHT_IN_VIEW_M = 0.18;

export interface AlertLevels {
  opacity: number;
  /** Extra height, in meters, while it settles into place. */
  rise: number;
}

const ease = (k: number): number => k * k * (3 - 2 * k);

/** Opacity and rise `t` seconds after triggering: a short fade in, a hold, a slower fade out. */
export function alertLevels(t: number): AlertLevels {
  if (t < 0 || t >= DURATION_S) return { opacity: 0, rise: 0 };
  if (t < FADE_IN_S) {
    const k = ease(t / FADE_IN_S);
    return { opacity: k, rise: RISE_M * (1 - k) };
  }
  if (t < FADE_IN_S + HOLD_S) return { opacity: 1, rise: 0 };
  return { opacity: 1 - ease((t - FADE_IN_S - HOLD_S) / FADE_OUT_S), rise: 0 };
}

function toastTexture(): CanvasTexture {
  const canvas = document.createElement('canvas');
  canvas.width = 768;
  canvas.height = 160;
  const ctx = canvas.getContext('2d')!;
  roundedRect(ctx, 4, 4, 760, 152, 30);
  ctx.fillStyle = 'rgba(16, 20, 24, 0.85)';
  ctx.fill();
  ctx.strokeStyle = 'rgba(255, 255, 255, 0.22)';
  ctx.lineWidth = 3;
  ctx.stroke();
  ctx.fillStyle = '#e74c3c'; // the boat pin's color
  ctx.beginPath();
  ctx.arc(74, 80, 14, 0, Math.PI * 2);
  ctx.fill();
  ctx.fillStyle = '#f2f5f7';
  ctx.font = '600 52px system-ui, -apple-system, sans-serif';
  ctx.textAlign = 'left';
  ctx.textBaseline = 'middle';
  ctx.fillText('Boat detected', 120, 84);
  const texture = new CanvasTexture(canvas);
  texture.colorSpace = SRGBColorSpace;
  return texture;
}

/**
 * A quiet notification shown in the wearer's view the first time the boat is found: a small card that
 * fades in, holds for a couple of seconds, and fades out. Add `group` to the head (`world.player.head`)
 * so it stays in view; it draws over everything, ignoring depth.
 */
export class BoatAlert {
  readonly group = new Group();
  private readonly toast: Mesh;
  private elapsed = Infinity;
  private handled = false;

  constructor() {
    this.toast = new Mesh(
      new PlaneGeometry(WIDTH_M, HEIGHT_M),
      new MeshBasicMaterial({ map: toastTexture(), transparent: true, depthTest: false, depthWrite: false, toneMapped: false }),
    );
    this.toast.position.set(0, HEIGHT_IN_VIEW_M, -DISTANCE_M);
    this.toast.renderOrder = 1000; // above the terrain, pins, labels and camera panels
    this.toast.frustumCulled = false;
    this.group.add(this.toast);
    this.group.visible = false;
  }

  /**
   * Call with each batch from the feed. Alerts once, for the first boat sighting, and only if that
   * sighting is recent: a page opened long after the mission found the boat stays quiet.
   */
  observe(assets: readonly FeedAsset[]): void {
    if (this.handled) return;
    const boat = assets.find((asset) => asset.kind === 'boat');
    if (!boat) return;
    this.handled = true;
    if ((boat.first_seen_age ?? 0) <= FRESH_S) this.trigger();
  }

  trigger(): void {
    this.elapsed = 0;
    this.group.visible = true;
    this.apply();
  }

  get active(): boolean {
    return this.elapsed < DURATION_S;
  }

  update(deltaSeconds: number): void {
    if (!this.active) return;
    this.elapsed += deltaSeconds;
    this.apply();
  }

  private apply(): void {
    const levels = alertLevels(this.elapsed);
    (this.toast.material as MeshBasicMaterial).opacity = levels.opacity;
    this.toast.position.y = HEIGHT_IN_VIEW_M + levels.rise;
    this.group.visible = this.active;
  }
}
