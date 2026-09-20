import { CanvasTexture, Group, Mesh, MeshBasicMaterial, PlaneGeometry, SRGBColorSpace } from '@iwsdk/core';
import { roundedRect } from './canvas-util';

export type HudState = 'listening' | 'thinking' | 'answer' | 'notice' | 'error';

const CANVAS_W = 1024;
const CANVAS_H = 320;
const WIDTH_M = 0.6;
const HEIGHT_M = (WIDTH_M * CANVAS_H) / CANVAS_W;
const DISTANCE_M = 1.0;
const HEIGHT_IN_VIEW_M = 0.36; // above the boat alert (0.18), which is above the tabletop map
const FADE_IN_S = 0.2;
const FADE_OUT_S = 0.5;
const PAD = 40;
const BODY_FONT_PX = 34;
const LINE_HEIGHT = 44;
const MAX_LINES = 5;

const STYLE: Record<HudState, { dot: string; label: string }> = {
  listening: { dot: '#e74c3c', label: 'LISTENING' },
  thinking: { dot: '#f1c40f', label: 'THINKING' },
  answer: { dot: '#2ecc71', label: 'ASSISTANT' },
  notice: { dot: '#9aa7b1', label: 'ASSISTANT' },
  error: { dot: '#e67e22', label: 'ASSISTANT' },
};

/** Break `text` into lines no wider than `maxWidth` by `measure`. Words are never split unless one alone is too wide. */
export function wrapText(text: string, maxWidth: number, measure: (s: string) => number): string[] {
  const lines: string[] = [];
  let line = '';
  for (const word of text.split(/\s+/).filter(Boolean)) {
    const candidate = line ? `${line} ${word}` : word;
    if (measure(candidate) <= maxWidth || !line) {
      line = candidate;
    } else {
      lines.push(line);
      line = word;
    }
  }
  if (line) lines.push(line);
  return lines;
}

/**
 * A card in the wearer's view for the voice assistant: what it is doing (listening, thinking) and
 * what it answered, as a caption to go with the voice. Add `group` to the head so it stays in view.
 * It draws over everything, ignoring depth.
 */
export class HudToast {
  readonly group = new Group();
  private readonly canvas = document.createElement('canvas');
  private readonly ctx = this.canvas.getContext('2d')!;
  private readonly texture: CanvasTexture;
  private readonly material: MeshBasicMaterial;
  private state: HudState | null = null;
  private text = '';
  private caption = ''; // what was heard, shown small beside the state label
  private fade = 0; // 0..1 opacity
  private wanted = false; // true while it should be showing
  private holdLeft: number | null = null; // seconds until it starts to leave; null = stay

  constructor() {
    this.canvas.width = CANVAS_W;
    this.canvas.height = CANVAS_H;
    this.texture = new CanvasTexture(this.canvas);
    this.texture.colorSpace = SRGBColorSpace;
    this.material = new MeshBasicMaterial({
      map: this.texture,
      transparent: true,
      opacity: 0,
      depthTest: false,
      depthWrite: false,
      toneMapped: false,
    });
    const mesh = new Mesh(new PlaneGeometry(WIDTH_M, HEIGHT_M), this.material);
    mesh.position.set(0, HEIGHT_IN_VIEW_M, -DISTANCE_M);
    mesh.renderOrder = 1001; // above the boat alert
    mesh.frustumCulled = false;
    this.group.add(mesh);
    this.group.visible = false;
  }

  get current(): { state: HudState | null; text: string; showing: boolean } {
    return { state: this.state, text: this.text, showing: this.wanted };
  }

  /** Show `text` in `state`. It stays until replaced or hidden, or for `holdSeconds` if given. */
  show(state: HudState, text: string, holdSeconds: number | null = null, caption = ''): void {
    this.state = state;
    this.text = text;
    this.caption = caption;
    this.wanted = true;
    this.holdLeft = holdSeconds;
    this.group.visible = true;
    this.draw();
  }

  /** Start leaving after `seconds` (0 = now), keeping whatever it shows. */
  hideAfter(seconds: number): void {
    if (this.wanted) this.holdLeft = seconds;
  }

  update(deltaSeconds: number): void {
    if (this.wanted && this.holdLeft !== null) {
      this.holdLeft -= deltaSeconds;
      if (this.holdLeft <= 0) {
        this.wanted = false;
        this.holdLeft = null;
      }
    }
    this.fade = this.wanted
      ? Math.min(1, this.fade + deltaSeconds / FADE_IN_S)
      : Math.max(0, this.fade - deltaSeconds / FADE_OUT_S);
    this.material.opacity = this.fade;
    this.group.visible = this.wanted || this.fade > 0;
  }

  private draw(): void {
    const { ctx } = this;
    const style = STYLE[this.state ?? 'notice'];
    ctx.clearRect(0, 0, CANVAS_W, CANVAS_H);
    roundedRect(ctx, 4, 4, CANVAS_W - 8, CANVAS_H - 8, 34);
    ctx.fillStyle = 'rgba(16, 20, 24, 0.86)';
    ctx.fill();
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.22)';
    ctx.lineWidth = 3;
    ctx.stroke();

    ctx.fillStyle = style.dot;
    ctx.beginPath();
    ctx.arc(PAD + 10, 56, 10, 0, Math.PI * 2);
    ctx.fill();
    ctx.textBaseline = 'middle';
    ctx.textAlign = 'left';
    ctx.font = '600 24px system-ui, -apple-system, sans-serif';
    ctx.fillStyle = '#9aa7b1';
    const labelX = PAD + 34;
    ctx.fillText(style.label, labelX, 57);
    if (this.caption) {
      // What was heard, so a misheard question is obvious. Trimmed to fit the row.
      const room = CANVAS_W - PAD - (labelX + ctx.measureText(style.label).width + 24);
      let heard = `\u201c${this.caption}\u201d`;
      while (heard.length > 4 && ctx.measureText(heard).width > room) heard = `${heard.slice(0, -2)}\u2026`;
      ctx.fillStyle = '#c9d2d9';
      ctx.fillText(heard, labelX + ctx.measureText(style.label).width + 24, 57);
    }

    ctx.font = `500 ${BODY_FONT_PX}px system-ui, -apple-system, sans-serif`;
    ctx.fillStyle = '#f2f5f7';
    let lines = wrapText(this.text, CANVAS_W - 2 * PAD, (s) => ctx.measureText(s).width);
    if (lines.length > MAX_LINES) lines = [...lines.slice(0, MAX_LINES - 1), `${lines[MAX_LINES - 1]}…`];
    lines.forEach((line, i) => ctx.fillText(line, PAD, 118 + i * LINE_HEIGHT));
    this.texture.needsUpdate = true;
  }
}
