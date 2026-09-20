export type AssetKind = 'copter' | 'plane' | 'tower' | 'boat';

export interface FeedAsset {
  name: string;
  kind: AssetKind;
  lat: number;
  lon: number;
  /** Altitude above mean sea level, meters. The sim keeps this equal to world z. */
  alt: number;
  /** Seconds since the bridge last heard from this asset. */
  age: number;
}

export type FeedStatus = 'connecting' | 'live' | 'offline';

/**
 * Polls the position bridge (webxr/bridge/feed.py) for asset positions.
 *
 * Default is same-origin `/feed/positions`, which vite proxies to the bridge on the dev machine.
 * That avoids mixed-content blocking (https page, http bridge) and works from the headset over
 * the LAN. Override with `?feed=<url>` or VITE_FEED_URL.
 */
export class TelemetryFeed {
  status: FeedStatus = 'connecting';
  assets: FeedAsset[] = [];
  private timer: number | undefined;
  private failures = 0;

  constructor(
    private readonly url: string = new URLSearchParams(location.search).get('feed') ??
      import.meta.env.VITE_FEED_URL ??
      '/feed/positions',
    private readonly intervalMs = 250,
    private readonly onUpdate: (assets: FeedAsset[]) => void = () => {},
    private readonly onStatus: (status: FeedStatus) => void = () => {},
  ) {}

  start(): void {
    if (this.timer !== undefined) return;
    void this.poll();
    this.timer = window.setInterval(() => void this.poll(), this.intervalMs);
  }

  stop(): void {
    if (this.timer !== undefined) window.clearInterval(this.timer);
    this.timer = undefined;
  }

  private setStatus(status: FeedStatus): void {
    if (status === this.status) return;
    this.status = status;
    this.onStatus(status);
  }

  private async poll(): Promise<void> {
    try {
      const response = await fetch(this.url, { cache: 'no-store' });
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      const body = (await response.json()) as { assets?: FeedAsset[] };
      this.assets = body.assets ?? [];
      this.failures = 0;
      this.setStatus('live');
      this.onUpdate(this.assets);
    } catch {
      // Tolerate a few dropped polls before calling the feed offline.
      if (++this.failures >= 8) this.setStatus('offline');
    }
  }
}
