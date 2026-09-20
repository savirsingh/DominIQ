/**
 * MJPEG over HTTP, as served by arctic-sim's CameraStreamPlugin:
 *
 *   Content-Type: multipart/x-mixed-replace; boundary=arcticframe
 *   --arcticframe\r\nContent-Type: image/jpeg\r\nContent-Length: N\r\n\r\n<N bytes>\r\n ...
 *
 * Parsed by hand with fetch instead of pointing an <img> at it: it works the same in a detached
 * page, can be cancelled, reconnects on its own, and is testable outside a browser.
 */

export type StreamStatus = 'connecting' | 'live' | 'offline';

export interface FrameSource {
  close(): void;
}

export type OpenStream = (
  url: string,
  onFrame: (jpeg: Uint8Array) => void,
  onStatus: (status: StreamStatus) => void,
) => FrameSource;

const HEADER_END = [13, 10, 13, 10]; // \r\n\r\n
const MAX_BUFFER = 8 * 1024 * 1024; // a real frame is tens of KB; more than this means we lost sync
const LENGTH = /content-length:\s*(\d+)/i;

function indexOfHeaderEnd(buf: Uint8Array): number {
  for (let i = 0; i + 3 < buf.length; i++) {
    if (buf[i] === HEADER_END[0] && buf[i + 1] === HEADER_END[1] && buf[i + 2] === HEADER_END[2] && buf[i + 3] === HEADER_END[3]) {
      return i;
    }
  }
  return -1;
}

/** Feed it arbitrary chunks of the response body; it returns each complete JPEG as it finishes. */
export class MjpegParser {
  private buf = new Uint8Array(0);

  push(chunk: Uint8Array): Uint8Array[] {
    const joined = new Uint8Array(this.buf.length + chunk.length);
    joined.set(this.buf);
    joined.set(chunk, this.buf.length);
    this.buf = joined;

    const frames: Uint8Array[] = [];
    for (;;) {
      const end = indexOfHeaderEnd(this.buf);
      if (end < 0) break;
      const header = new TextDecoder().decode(this.buf.subarray(0, end));
      const match = LENGTH.exec(header);
      if (!match) {
        this.buf = this.buf.slice(end + 4); // a part with no length: skip its headers and resync
        continue;
      }
      const start = end + 4;
      const stop = start + Number(match[1]);
      if (this.buf.length < stop) break; // the rest of this frame has not arrived yet
      frames.push(this.buf.slice(start, stop));
      this.buf = this.buf.slice(stop);
    }
    if (this.buf.length > MAX_BUFFER) this.buf = new Uint8Array(0);
    return frames;
  }
}

/** Open a stream and keep it open: reconnects with backoff (1 s up to 5 s) until closed. */
export const openMjpeg: OpenStream = (url, onFrame, onStatus) => {
  const abort = new AbortController();
  let closed = false;

  void (async () => {
    let delayMs = 1000;
    while (!closed) {
      onStatus('connecting');
      try {
        const response = await fetch(url, { signal: abort.signal, cache: 'no-store' });
        if (!response.ok || !response.body) throw new Error(`HTTP ${response.status}`);
        const reader = response.body.getReader();
        const parser = new MjpegParser();
        for (;;) {
          const { done, value } = await reader.read();
          if (done) break;
          for (const frame of parser.push(value)) {
            delayMs = 1000;
            onStatus('live');
            onFrame(frame);
          }
        }
      } catch {
        if (closed) return;
      }
      if (closed) return;
      onStatus('offline');
      await new Promise((resolve) => setTimeout(resolve, delayMs));
      delayMs = Math.min(delayMs * 2, 5000);
    }
  })();

  return {
    close() {
      closed = true;
      abort.abort();
    },
  };
};
