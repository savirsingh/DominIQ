/**
 * Push-to-talk voice assistant client. Hold the talk button and speak, release to send. The recording goes
 * to the assistant service (assistant/server.py, through the dev server's /ask proxy), which does
 * Whisper -> GPT-4o -> ElevenLabs and returns the answer text and its spoken mp3. The keys never reach
 * the headset.
 *
 * Everything the browser provides (microphone, recorder, audio element, fetch) is injectable, so the
 * logic can be tested without a headset.
 */

export interface Focus {
  hovered: string | null;
  locked: string[];
}

export type VoiceEvent =
  | { kind: 'listening' }
  | { kind: 'thinking' }
  | { kind: 'answer'; question: string; answer: string; speaking: boolean }
  | { kind: 'speech-ended' }
  | { kind: 'cancelled'; reason: string }
  | { kind: 'error'; message: string };

export interface RecorderLike {
  start(): void;
  stop(): Promise<Blob>;
}

export interface PlayerLike {
  /** Call from a user gesture: browsers only allow audio after one. */
  unlock(): void;
  /** Resolves when playback finishes or is stopped. */
  play(audio: Blob): Promise<void>;
  stop(): void;
}

export interface VoiceDeps {
  fetch?: typeof fetch;
  getStream?: () => Promise<MediaStream>;
  makeRecorder?: (stream: MediaStream) => RecorderLike;
  player?: PlayerLike;
  now?: () => number;
}

export interface VoiceOptions {
  /** The assistant's /ask endpoint. */
  url: string;
  getFocus: () => Focus;
  onEvent: (event: VoiceEvent) => void;
  /** A press shorter than this is treated as a mistake and sent nowhere. */
  minHoldMs?: number;
  /** A recording smaller than this is silence or a click. */
  minBytes?: number;
  timeoutMs?: number;
}

const RECORDER_TYPES = ['audio/webm;codecs=opus', 'audio/webm', 'audio/mp4', 'audio/ogg;codecs=opus'];

class BrowserRecorder implements RecorderLike {
  private readonly recorder: MediaRecorder;
  private readonly chunks: Blob[] = [];

  constructor(stream: MediaStream) {
    const type = RECORDER_TYPES.find((t) => MediaRecorder.isTypeSupported(t));
    this.recorder = new MediaRecorder(stream, type ? { mimeType: type } : undefined);
    this.recorder.ondataavailable = (e) => e.data.size > 0 && this.chunks.push(e.data);
  }

  start(): void {
    this.recorder.start();
  }

  stop(): Promise<Blob> {
    return new Promise((resolve) => {
      this.recorder.onstop = () => resolve(new Blob(this.chunks, { type: this.recorder.mimeType || 'audio/webm' }));
      this.recorder.stop();
    });
  }
}

// A one-sample silent WAV, played inside a user gesture to unlock the audio element for later.
const SILENCE = 'data:audio/wav;base64,UklGRiQAAABXQVZFZm10IBAAAAABAAEAQB8AAIA+AAACABAAZGF0YQAAAAA=';

class BrowserPlayer implements PlayerLike {
  private readonly audio = new Audio();
  private finish: (() => void) | null = null;

  unlock(): void {
    this.audio.src = SILENCE;
    void this.audio.play().catch(() => {});
  }

  play(blob: Blob): Promise<void> {
    this.stop();
    const url = URL.createObjectURL(blob);
    return new Promise((resolve) => {
      const done = (): void => {
        this.finish = null;
        this.audio.onended = this.audio.onerror = null;
        URL.revokeObjectURL(url);
        resolve();
      };
      this.finish = done;
      this.audio.onended = done;
      this.audio.onerror = done;
      this.audio.src = url;
      this.audio.play().catch(done);
    });
  }

  stop(): void {
    this.audio.pause();
    this.finish?.();
  }
}

function base64ToBlob(b64: string, mime: string): Blob {
  const bytes = atob(b64);
  const data = new Uint8Array(bytes.length);
  for (let i = 0; i < bytes.length; i++) data[i] = bytes.charCodeAt(i);
  return new Blob([data], { type: mime });
}

export class VoiceAssistant {
  private readonly fetchFn: typeof fetch;
  private readonly getStream: () => Promise<MediaStream>;
  private readonly makeRecorder: (stream: MediaStream) => RecorderLike;
  private readonly player: PlayerLike;
  private readonly now: () => number;
  private stream: MediaStream | null = null;
  private streamPromise: Promise<MediaStream> | null = null;
  private recorder: RecorderLike | null = null;
  private pressed = false;
  private pressedAt = 0;
  private request: AbortController | null = null;
  private generation = 0; // bumped by every press, so the answer to an interrupted question is dropped

  constructor(
    private readonly options: VoiceOptions,
    deps: VoiceDeps = {},
  ) {
    this.fetchFn = deps.fetch ?? ((...args) => fetch(...args));
    this.getStream = deps.getStream ?? (() => navigator.mediaDevices.getUserMedia({ audio: true }));
    this.makeRecorder = deps.makeRecorder ?? ((stream) => new BrowserRecorder(stream));
    this.player = deps.player ?? new BrowserPlayer();
    this.now = deps.now ?? (() => performance.now());
  }

  /**
   * Get microphone access and unlock audio playback. Call from the click that enters AR, before the
   * immersive session starts: a permission prompt may not be usable inside one. Never throws.
   */
  async prepare(): Promise<boolean> {
    this.player.unlock();
    try {
      await this.ensureStream();
      return true;
    } catch {
      return false;
    }
  }

  private ensureStream(): Promise<MediaStream> {
    if (this.stream) return Promise.resolve(this.stream);
    this.streamPromise ??= this.getStream().then(
      (stream) => {
        stream.getAudioTracks().forEach((track) => (track.enabled = false)); // not listening until asked
        this.stream = stream;
        return stream;
      },
      (error) => {
        this.streamPromise = null; // let the next press try again
        throw error;
      },
    );
    return this.streamPromise;
  }

  /** The talk button went down. */
  async press(): Promise<void> {
    const generation = ++this.generation;
    this.pressed = true;
    this.pressedAt = this.now();
    this.player.stop(); // talking over the assistant interrupts it
    void this.recorder?.stop(); // a press with no release in between: drop the old recording
    this.recorder = null;
    this.request?.abort();
    this.request = null;
    this.options.onEvent({ kind: 'listening' });

    let stream: MediaStream;
    try {
      stream = await this.ensureStream();
    } catch {
      if (generation === this.generation) {
        this.pressed = false;
        this.options.onEvent({ kind: 'error', message: 'I need microphone access. Allow it for this page and try again.' });
      }
      return;
    }
    if (generation !== this.generation || !this.pressed) return; // released (or pressed again) while waiting
    stream.getAudioTracks().forEach((track) => (track.enabled = true));
    this.recorder = this.makeRecorder(stream);
    this.recorder.start();
  }

  /** The talk button came up. */
  async release(): Promise<void> {
    if (!this.pressed) return;
    this.pressed = false;
    const generation = this.generation;
    const heldMs = this.now() - this.pressedAt;
    const recorder = this.recorder;
    this.recorder = null;
    this.stream?.getAudioTracks().forEach((track) => (track.enabled = false));

    if (!recorder) {
      this.options.onEvent({ kind: 'cancelled', reason: 'Hold the button and speak.' });
      return;
    }
    const audio = await recorder.stop();
    if (generation !== this.generation) return;
    if (heldMs < (this.options.minHoldMs ?? 400) || audio.size < (this.options.minBytes ?? 1500)) {
      this.options.onEvent({ kind: 'cancelled', reason: 'Hold the button and speak.' });
      return;
    }
    void this.ask(audio, generation); // runs on in the background; ask() reports through onEvent and never throws
  }

  private async ask(audio: Blob, generation: number): Promise<void> {
    this.options.onEvent({ kind: 'thinking' });
    const controller = new AbortController();
    this.request = controller;
    const timeout = setTimeout(() => controller.abort(), this.options.timeoutMs ?? 40000);
    try {
      const response = await this.fetchFn(this.options.url, {
        method: 'POST',
        body: audio,
        headers: { 'Content-Type': audio.type || 'audio/webm', 'X-Focus': encodeURIComponent(JSON.stringify(this.options.getFocus())) },
        signal: controller.signal,
      });
      const body = (await response.json().catch(() => ({}))) as Record<string, any>;
      if (generation !== this.generation) return;
      if (!response.ok) {
        this.options.onEvent({ kind: 'error', message: this.describeFailure(response.status, body) });
        return;
      }
      const speaking = Boolean(body.audio_b64);
      this.options.onEvent({ kind: 'answer', question: String(body.question ?? ''), answer: String(body.answer ?? ''), speaking });
      if (speaking) void this.speak(body.audio_b64, body.audio_mime || 'audio/mpeg', generation); // plays on; we are done
    } catch (error) {
      if (generation !== this.generation) return; // interrupted on purpose
      const timedOut = (error as Error)?.name === 'AbortError';
      this.options.onEvent({
        kind: 'error',
        message: timedOut
          ? 'The assistant took too long to answer.'
          : "I can't reach the assistant. Is assistant/server.py running?",
      });
    } finally {
      clearTimeout(timeout);
      if (this.request === controller) this.request = null;
    }
  }

  /** Play the spoken answer. Runs on after release() returns; a newer press cuts it off and silences its "ended". */
  private async speak(b64: string, mime: string, generation: number): Promise<void> {
    await this.player.play(base64ToBlob(b64, mime));
    if (generation === this.generation) this.options.onEvent({ kind: 'speech-ended' });
  }

  private describeFailure(status: number, body: Record<string, any>): string {
    if (status === 422) return "I didn't catch that. Try again.";
    if (status === 503 && body.error) return `Assistant not set up: ${body.error}.`;
    if (body.stage === 'answer') return "I heard you but couldn't get an answer.";
    if (body.stage === 'transcribe') return "I couldn't make out what you said.";
    return `The assistant had a problem (${status}).`;
  }
}
