export type TranscriberStatus = 'idle' | 'listening' | 'processing' | 'done' | 'error';

type StatusCallback = (status: TranscriberStatus, message?: string) => void;

export class SpeechTranscriber {
  private apiKey: string;
  private onStatus: StatusCallback;
  private recorder: MediaRecorder | null = null;
  private chunks: Blob[] = [];
  private _active = false;
  private starting = false;

  constructor(apiKey: string, onStatus: StatusCallback) {
    this.apiKey = apiKey;
    this.onStatus = onStatus;
  }

  get active() {
    return this._active;
  }

  async startListening() {
    if (this._active || this.starting) return;
    this.starting = true;
    try {
      const stream = await navigator.mediaDevices.getUserMedia({ audio: true });
      this.chunks = [];
      const mimeType = MediaRecorder.isTypeSupported('audio/webm;codecs=opus')
        ? 'audio/webm;codecs=opus'
        : 'audio/webm';
      this.recorder = new MediaRecorder(stream, { mimeType });
      this.recorder.ondataavailable = (e) => {
        if (e.data.size > 0) this.chunks.push(e.data);
      };
      this.recorder.start(100);
      this._active = true;
      this.onStatus('listening');
    } catch {
      this.onStatus('error', 'Mic access denied');
    } finally {
      this.starting = false;
    }
  }

  async stopListening() {
    if (!this._active || !this.recorder) return;

    await new Promise<void>((resolve) => {
      this.recorder!.onstop = () => resolve();
      this.recorder!.stop();
    });

    this._active = false;
    this.recorder.stream.getTracks().forEach((t) => t.stop());

    if (this.chunks.length === 0) {
      this.onStatus('idle');
      return;
    }

    this.onStatus('processing');
    const blob = new Blob(this.chunks, { type: this.chunks[0].type });
    const text = await this.transcribe(blob);

    if (text) {
      this.onStatus('done', text);
    } else {
      this.onStatus('error', 'No speech detected');
    }
  }

  private async transcribe(audio: Blob): Promise<string | null> {
    try {
      const form = new FormData();
      form.append('file', audio, 'speech.webm');
      form.append('model_id', 'scribe_v1');
      form.append('language_code', 'eng');

      const res = await fetch('https://api.elevenlabs.io/v1/speech-to-text', {
        method: 'POST',
        headers: { 'xi-api-key': this.apiKey },
        body: form,
      });

      if (!res.ok) {
        console.error('ElevenLabs STT error:', res.status);
        return null;
      }

      const data = await res.json();
      return (data.text ?? '').trim() || null;
    } catch (err) {
      console.error('STT failed:', err);
      return null;
    }
  }
}
