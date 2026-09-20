import { CanvasTexture, Group, Mesh, MeshBasicMaterial, PlaneGeometry, SRGBColorSpace } from '@iwsdk/core';
import { openMjpeg, type FrameSource, type OpenStream, type StreamStatus } from './mjpeg';

const WIDTH_M = 0.36; // same width as the info label it sits on
const CANVAS_W = 640;
const CANVAS_H = 360; // 16:9. Frames of another shape are letterboxed
const STALE_MS = 3000; // no frame for this long while "live" means the feed has stalled

export type Decode = (jpeg: Blob) => Promise<ImageBitmap>;

/**
 * A floating panel showing one asset's live camera. Frames go onto a canvas (which also carries
 * the status overlay) and the canvas is the panel's texture. The stream is open only between
 * start() and stop().
 */
export class CameraPanel {
  readonly group = new Group();
  private readonly canvas = document.createElement('canvas');
  private readonly ctx = this.canvas.getContext('2d')!;
  private readonly texture: CanvasTexture;
  private source: FrameSource | null = null;
  private status: StreamStatus = 'connecting';
  private lastFrameAt = 0;
  private decoding = false;
  private pending: Uint8Array | null = null;
  private bitmap: ImageBitmap | null = null;
  private stalled = false;
  private generation = 0;

  constructor(
    private readonly title: string,
    private readonly open: OpenStream = openMjpeg,
    private readonly decode: Decode = (blob) => createImageBitmap(blob),
  ) {
    this.canvas.width = CANVAS_W;
    this.canvas.height = CANVAS_H;
    this.texture = new CanvasTexture(this.canvas);
    this.texture.colorSpace = SRGBColorSpace;

    const height = (WIDTH_M * CANVAS_H) / CANVAS_W;
    const mesh = new Mesh(
      new PlaneGeometry(WIDTH_M, height),
      new MeshBasicMaterial({ map: this.texture, toneMapped: false }),
    );
    mesh.position.y = height / 2; // the group's origin is the panel's bottom edge, like the label's
    this.group.add(mesh);
    this.group.visible = false;
    this.draw();
  }

  get height(): number {
    return (WIDTH_M * CANVAS_H) / CANVAS_W;
  }

  get streaming(): boolean {
    return this.source !== null;
  }

  start(url: string): void {
    if (this.source) return;
    const generation = ++this.generation;
    this.status = 'connecting';
    this.stalled = false;
    this.lastFrameAt = 0;
    this.draw();
    this.source = this.open(
      url,
      (jpeg) => {
        if (generation !== this.generation) return;
        this.lastFrameAt = performance.now();
        this.stalled = false;
        this.pending = jpeg; // only the newest frame is worth decoding
        void this.pump();
      },
      (status) => {
        if (generation !== this.generation || status === this.status) return;
        this.status = status;
        this.draw();
      },
    );
  }

  stop(): void {
    this.source?.close();
    this.source = null;
    this.generation++; // late frames and statuses from the old stream are ignored
    this.pending = null;
    this.bitmap?.close();
    this.bitmap = null;
    this.draw();
  }

  /** Call every frame while the panel is showing: notices a stalled feed. */
  update(): void {
    if (!this.source || this.status !== 'live' || this.stalled) return;
    if (this.lastFrameAt && performance.now() - this.lastFrameAt > STALE_MS) {
      this.stalled = true;
      this.draw();
    }
  }

  private async pump(): Promise<void> {
    if (this.decoding) return;
    this.decoding = true;
    const generation = this.generation;
    try {
      while (this.pending && generation === this.generation) {
        const jpeg = this.pending;
        this.pending = null;
        try {
          const bitmap = await this.decode(new Blob([jpeg as BlobPart], { type: 'image/jpeg' }));
          if (generation !== this.generation) {
            bitmap.close();
            break;
          }
          this.bitmap?.close();
          this.bitmap = bitmap;
          this.draw();
        } catch {
          // A corrupt frame is dropped; the next one replaces it.
        }
      }
    } finally {
      this.decoding = false;
    }
  }

  private draw(): void {
    const { ctx } = this;
    ctx.fillStyle = '#05080a';
    ctx.fillRect(0, 0, CANVAS_W, CANVAS_H);

    if (this.bitmap) {
      // Fit the frame inside the panel without stretching it.
      const scale = Math.min(CANVAS_W / this.bitmap.width, CANVAS_H / this.bitmap.height);
      const w = this.bitmap.width * scale;
      const h = this.bitmap.height * scale;
      ctx.drawImage(this.bitmap, (CANVAS_W - w) / 2, (CANVAS_H - h) / 2, w, h);
    }

    const live = this.status === 'live' && !this.stalled && this.bitmap !== null;
    const message = live
      ? null
      : this.stalled
        ? 'NO SIGNAL'
        : this.status === 'offline'
          ? 'CAMERA OFFLINE - RETRYING'
          : this.source
            ? 'CONNECTING...'
            : null;
    if (message) {
      ctx.fillStyle = 'rgba(0, 0, 0, 0.55)';
      ctx.fillRect(0, CANVAS_H / 2 - 28, CANVAS_W, 56);
      ctx.fillStyle = '#ffd54a';
      ctx.font = 'bold 26px monospace';
      ctx.textAlign = 'center';
      ctx.fillText(message, CANVAS_W / 2, CANVAS_H / 2 + 9);
    }

    // Title tag, top left, with a status dot.
    ctx.textAlign = 'left';
    ctx.font = 'bold 22px monospace';
    const tag = this.title.toUpperCase();
    ctx.fillStyle = 'rgba(0, 0, 0, 0.55)';
    ctx.fillRect(10, 10, ctx.measureText(tag).width + 46, 34);
    ctx.fillStyle = live ? '#2ecc71' : '#e74c3c';
    ctx.beginPath();
    ctx.arc(28, 27, 7, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = '#ffffff';
    ctx.fillText(tag, 44, 35);

    ctx.strokeStyle = 'rgba(255, 255, 255, 0.85)';
    ctx.lineWidth = 3;
    ctx.strokeRect(1.5, 1.5, CANVAS_W - 3, CANVAS_H - 3);
    this.texture.needsUpdate = true;
  }
}
