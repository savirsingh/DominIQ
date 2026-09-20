#!/usr/bin/env python3
"""Voice assistant sidecar for the WebXR view: mic -> Whisper -> GPT-4o -> ElevenLabs, as an HTTP service.

    python3 assistant/server.py              # real services; needs OPENAI_API_KEY and ELEVENLABS_API_KEY in .env
    python3 assistant/server.py --demo       # canned answers and a beep; no keys, no cost

The headset records audio and POSTs it here (the vite dev server proxies /ask). The service transcribes it,
answers using the mission's live state, and replies with the answer text and the spoken mp3.

It runs as its OWN process, not inside mission.py, so a slow or failing API call can never touch the
mission's flight and detection threads. It only READS the mission's feed (mission.py --webxr-feed) and,
if present, the mission log; it does not talk MAVLink.

    POST /ask   Content-Type: audio/webm (or ogg, mp4, wav): the recorded question. Optional header
                X-Focus: URL-encoded JSON {"hovered": "tower-1", "locked": ["quadcopter"]}
    POST /ask   Content-Type: application/json: {"text": "where is the boat", "focus": {...}}  (skips Whisper)
      200 {"question", "answer", "audio_b64", "audio_mime"}     audio_* are null if speech synthesis failed
      4xx/5xx {"error", "stage": "request|transcribe|answer"}
    GET /health
"""
from __future__ import annotations

import argparse
import base64
import json
import os
import socketserver
import sys
import threading
import time
import urllib.parse
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from context import build_context, build_system_prompt, read_log_state  # noqa: E402
from core import Backends, MissingKey, real_backends  # noqa: E402

DEFAULT_PORT = 8782
DEFAULT_FEED = "http://127.0.0.1:8781/positions"
DEFAULT_LOG = "/tmp/mission_out.log"
MAX_BODY = 8 * 1024 * 1024  # ten seconds of opus is ~100 KB; this is a generous ceiling


def fetch_snapshot(url: str, timeout: float = 1.5) -> Optional[dict]:
    try:
        with urllib.request.urlopen(url, timeout=timeout) as response:
            return json.load(response)
    except Exception:  # the mission may simply not be running
        return None


class _Server(ThreadingHTTPServer):
    def server_bind(self) -> None:
        # HTTPServer.server_bind() does a reverse-DNS lookup of our own address (socket.getfqdn), which can
        # take seconds on macOS. Nothing here uses the name.
        socketserver.TCPServer.server_bind(self)
        self.server_name, self.server_port = str(self.server_address[0]), self.server_address[1]


def make_server(backends: Backends, feed_url: str = DEFAULT_FEED, log_path: Optional[str] = DEFAULT_LOG,
                host: str = "127.0.0.1", port: int = DEFAULT_PORT) -> ThreadingHTTPServer:

    class Handler(BaseHTTPRequestHandler):
        def _send(self, status: int, payload: dict) -> None:
            body = json.dumps(payload).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *args) -> None:
            pass

        def do_OPTIONS(self) -> None:  # noqa: N802  (CORS preflight, for calling it directly from a page)
            self.send_response(204)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "POST, GET, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type, X-Focus")
            self.end_headers()

        def do_GET(self) -> None:  # noqa: N802
            if self.path.split("?", 1)[0] != "/health":
                return self._send(404, {"error": "not found", "stage": "request"})
            self._send(200, {"ok": True, "feed": fetch_snapshot(feed_url) is not None,
                             "log": bool(log_path and os.path.exists(log_path))})

        def do_POST(self) -> None:  # noqa: N802
            if self.path.split("?", 1)[0] != "/ask":
                return self._send(404, {"error": "not found", "stage": "request"})
            started = time.monotonic()
            try:
                length = int(self.headers.get("Content-Length", "0"))
            except ValueError:
                return self._send(400, {"error": "bad Content-Length", "stage": "request"})
            if length <= 0:
                return self._send(400, {"error": "empty request", "stage": "request"})
            if length > MAX_BODY:
                return self._send(413, {"error": "recording too large", "stage": "request"})
            body = self.rfile.read(length)
            ctype = (self.headers.get("Content-Type") or "").split(";")[0].strip().lower()
            mime = self.headers.get("Content-Type") or ctype

            focus = None
            question = ""
            try:
                if ctype == "application/json":
                    data = json.loads(body)
                    question = str(data.get("text", "")).strip()
                    focus = data.get("focus")
                elif ctype.startswith("audio/") or ctype == "video/webm":
                    raw = self.headers.get("X-Focus")
                    try:
                        focus = json.loads(urllib.parse.unquote(raw)) if raw else None
                    except ValueError:
                        focus = None
                else:
                    return self._send(415, {"error": f"unsupported Content-Type {ctype!r}", "stage": "request"})
            except (ValueError, AttributeError):
                return self._send(400, {"error": "could not parse the request", "stage": "request"})
            if not isinstance(focus, dict):
                focus = None

            if ctype != "application/json":
                try:
                    question = backends.transcribe(body, mime).strip()
                except MissingKey as exc:
                    return self._send(503, {"error": str(exc), "stage": "transcribe"})
                except Exception as exc:
                    print(f"[ask] transcription failed: {exc}", file=sys.stderr)
                    return self._send(502, {"error": f"transcription failed: {exc}", "stage": "transcribe"})
            if not question:
                return self._send(422, {"error": "no speech heard", "stage": "transcribe"})
            heard = time.monotonic()

            prompt = build_system_prompt(build_context(
                fetch_snapshot(feed_url), focus, read_log_state(log_path) if log_path else None))
            try:
                answer = backends.answer(question, prompt).strip()
            except MissingKey as exc:
                return self._send(503, {"error": str(exc), "stage": "answer", "question": question})
            except Exception as exc:
                print(f"[ask] answer failed: {exc}", file=sys.stderr)
                return self._send(502, {"error": f"answer failed: {exc}", "stage": "answer", "question": question})
            answered = time.monotonic()

            # A failed voice must not lose the answer: the headset shows the text either way.
            audio, audio_mime, speak_error = None, None, None
            try:
                spoken = backends.synthesize(answer)
                if spoken:
                    audio, audio_mime = base64.b64encode(spoken[0]).decode(), spoken[1]
            except Exception as exc:
                speak_error = str(exc)
                print(f"[ask] speech failed: {exc}", file=sys.stderr)

            print(f"[ask] Q: {question}\n      A: {answer}\n      "
                  f"whisper {heard - started:.1f}s, gpt {answered - heard:.1f}s, voice {time.monotonic() - answered:.1f}s")
            payload = {"question": question, "answer": answer, "audio_b64": audio, "audio_mime": audio_mime}
            if speak_error:
                payload["speak_error"] = speak_error
            self._send(200, payload)

    return _Server((host, port), Handler)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--demo", action="store_true", help="canned answers and a beep; needs no API keys")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--bind", default="127.0.0.1",
                    help="interface to listen on. Default is this machine only: /ask spends API credits, so it "
                         "should not be reachable from the LAN")
    ap.add_argument("--feed", default=DEFAULT_FEED, help="the mission's position feed (default: %(default)s)")
    ap.add_argument("--log", default=DEFAULT_LOG, help="mission log to read status from, if it exists (default: %(default)s)")
    args = ap.parse_args()

    try:
        from dotenv import load_dotenv
        load_dotenv()
    except ImportError:
        pass

    if args.demo:
        from demo import demo_backends
        backends = demo_backends()
        print("DEMO MODE: canned answers and a beep. No API keys are used.")
    else:
        backends = real_backends()
        missing = [k for k in ("OPENAI_API_KEY", "ELEVENLABS_API_KEY") if not os.environ.get(k)]
        if missing:
            print(f"Warning: {', '.join(missing)} not set (looked in the environment and .env). "
                  "Questions will fail until it is; use --demo to try the flow without keys.", file=sys.stderr)

    server = make_server(backends, args.feed, args.log, args.bind, args.port)
    print(f"Assistant listening on http://{args.bind}:{args.port}/ask")
    print(f"  live state from {args.feed}" + (f", log {args.log}" if args.log else ""))
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
