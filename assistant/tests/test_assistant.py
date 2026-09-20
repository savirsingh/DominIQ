"""Run:  python3 -m unittest discover -s assistant/tests -v      (no API keys or network needed)"""
import base64
import io
import json
import os
import sys
import tempfile
import threading
import unittest
import urllib.error
import urllib.request
import wave
from types import SimpleNamespace
from unittest import mock

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.dirname(HERE))                      # assistant/
sys.path.insert(0, os.path.join(ROOT, "webxr", "bridge"))       # the real position feed, for the integration test

import context  # noqa: E402
import core  # noqa: E402
import server  # noqa: E402
from demo import demo_backends  # noqa: E402
from feed import Feed, serve  # noqa: E402


def metres_north(lat, metres):
    return lat + metres / 111194.93


class ContextTests(unittest.TestCase):
    def test_haversine_and_bearing(self):
        self.assertAlmostEqual(context.haversine_m(0, 0, 1, 0), 111195, delta=5)
        self.assertAlmostEqual(context.bearing_deg(0, 0, 1, 0), 0, delta=0.01)
        self.assertAlmostEqual(context.bearing_deg(0, 0, 0, 1), 90, delta=0.01)
        self.assertAlmostEqual(context.bearing_deg(0, 0, -1, 0), 180, delta=0.01)
        self.assertAlmostEqual(context.bearing_deg(0, 0, 0, -1), 270, delta=0.01)

    def test_compass_words_and_distances(self):
        self.assertEqual([context.compass(b) for b in (0, 45, 90, 180, 270, 350, 359.9)],
                         ["north", "northeast", "east", "south", "west", "north", "north"])
        self.assertEqual(context.fmt_distance(850), "850 m")
        self.assertEqual(context.fmt_distance(1500), "1.5 km")

    def snapshot(self, **boat):
        assets = [
            {"name": "quadcopter", "kind": "copter", "lat": 72.0, "lon": -94.8, "alt": 150.0, "age": 0.2},
            {"name": "tower-1", "kind": "tower", "lat": metres_north(72.0, 500), "lon": -94.8, "alt": 115.0, "age": 0.3},
        ]
        if boat:
            assets.append({"name": "boat", "kind": "boat", "lat": metres_north(72.0, 1000), "lon": -94.8, "alt": 0.0, "age": 0.1, **boat})
        return {"t": 0, "assets": assets}

    def test_boat_is_described_relative_to_every_asset(self):
        text = context.build_context(self.snapshot(first_seen_age=42.0))
        self.assertIn("From quadcopter, the boat is 1.0 km to the north (0 degrees).", text)
        self.assertIn("From tower-1, the boat is 500 m to the north (0 degrees).", text)
        self.assertIn("first detected 42 s ago", text)
        self.assertIn("estimate", text)                       # the model must not treat it as ground truth
        self.assertIn("quadcopter to tower-1: 500 m to the north", text)

    def test_missing_pieces_are_stated_not_invented(self):
        self.assertIn("unavailable", context.build_context(None))
        self.assertIn("still be taking off", context.build_context({"assets": []}))
        self.assertIn("Boat: not detected yet", context.build_context(self.snapshot()))

    def test_stale_assets_are_flagged(self):
        snap = self.snapshot()
        snap["assets"][0]["age"] = 12.0
        self.assertIn("no update for 12 s", context.build_context(snap))

    def test_focus_lines(self):
        text = context.build_context(self.snapshot(), {"hovered": "tower-1", "locked": ["quadcopter", "boat"]})
        self.assertIn("pointing at tower-1", text)
        self.assertIn("quadcopter, boat", text)

    def test_log_state_matches_what_voice_py_reads(self):
        with tempfile.NamedTemporaryFile("w", suffix=".log", delete=False) as f:
            f.write("[10:00:00] [quadcopter  ] taking off\n"
                    "[10:00:01] --- STATUS ---\n  quadcopter  ALT 50\n  tower-1 ok\n  BOAT(71.9,-94.8)\n\n"
                    "[10:00:02] [quadcopter  ] BOAT DETECTED conf=0.90 -> (71.9, -94.8)\n"
                    "[10:00:03] [tower-1     ] MOTION DETECTED! bearing 40\n"
                    "[10:00:04] [fixed-wing  ] ARM FAILED\n")
        try:
            state = context.read_log_state(f.name)
        finally:
            os.unlink(f.name)
        self.assertEqual(len(state["assets"]), 2)
        self.assertEqual(state["boat"], "BOAT(71.9,-94.8)")
        self.assertEqual((len(state["detections"]), len(state["motion"]), len(state["errors"])), (1, 1, 1))
        text = context.build_context(None, None, state)
        self.assertIn("Recent tower motion events", text)
        self.assertIsNone(context.read_log_state("/nonexistent/mission.log"))


class Fake:
    """Records what the service asks of each step, and can be told to fail."""

    def __init__(self):
        self.transcript = "  where is the boat  "
        self.fail = None
        self.calls = []
        self.prompt = None

    def backends(self):
        def transcribe(audio, mime):
            self.calls.append(("transcribe", audio, mime))
            if self.fail == "transcribe":
                raise RuntimeError("whisper down")
            if self.fail == "key":
                raise core.MissingKey("OPENAI_API_KEY is not set")
            return self.transcript

        def answer(question, prompt):
            self.calls.append(("answer", question))
            self.prompt = prompt
            if self.fail == "answer":
                raise RuntimeError("gpt down")
            return "  It is one kilometre north.  "

        def synthesize(text):
            self.calls.append(("synthesize", text))
            if self.fail == "speak":
                raise RuntimeError("voice down")
            return b"MP3DATA", "audio/mpeg"

        return core.Backends(transcribe, answer, synthesize)


class ServerTests(unittest.TestCase):
    def setUp(self):
        self.fake = Fake()
        self.feed = Feed()
        self.feed.update("quadcopter", "copter", 72.0, -94.8, 150.0)
        self.feed.update("boat", "boat", metres_north(72.0, 1000), -94.8, 0.0)
        self.feed_server = serve(self.feed, 0)
        self.feed_url = f"http://127.0.0.1:{self.feed_server.server_address[1]}/positions"
        self.start(self.feed_url)

    def start(self, feed_url, backends=None):
        self.server = server.make_server(backends or self.fake.backends(), feed_url, None, "127.0.0.1", 0)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.base = f"http://127.0.0.1:{self.server.server_address[1]}"
        self.addCleanup(self.server.shutdown)
        self.addCleanup(self.server.server_close)

    def tearDown(self):
        self.feed_server.shutdown()
        self.feed_server.server_close()

    def post(self, body=b"x", ctype="audio/webm", headers=None, path="/ask"):
        req = urllib.request.Request(self.base + path, data=body, method="POST",
                                     headers={"Content-Type": ctype, **(headers or {})})
        try:
            with urllib.request.urlopen(req, timeout=5) as r:
                return r.status, json.load(r)
        except urllib.error.HTTPError as e:
            return e.code, json.load(e)

    def test_audio_question_full_pipeline(self):
        status, body = self.post(b"\x1aE\xdf\xa3AUDIO", "audio/webm;codecs=opus")
        self.assertEqual(status, 200)
        self.assertEqual(body["question"], "where is the boat")
        self.assertEqual(body["answer"], "It is one kilometre north.")
        self.assertEqual(base64.b64decode(body["audio_b64"]), b"MP3DATA")
        self.assertEqual(body["audio_mime"], "audio/mpeg")
        kind, audio, mime = self.fake.calls[0]
        self.assertEqual((kind, audio, mime), ("transcribe", b"\x1aE\xdf\xa3AUDIO", "audio/webm;codecs=opus"))
        self.assertEqual([c[0] for c in self.fake.calls], ["transcribe", "answer", "synthesize"])
        self.assertEqual(self.fake.calls[2][1], "It is one kilometre north.")   # what is spoken is the trimmed answer

    def test_context_comes_from_the_real_bridge_feed(self):
        self.post(b"a")
        self.assertIn("From quadcopter, the boat is 1.0 km to the north", self.fake.prompt)
        self.assertIn("first detected", self.fake.prompt)                         # first_seen_age made the round trip
        self.assertIn("AI operator assistant", self.fake.prompt)

    def test_text_question_skips_whisper(self):
        status, body = self.post(json.dumps({"text": "status?"}).encode(), "application/json")
        self.assertEqual(status, 200)
        self.assertEqual(body["question"], "status?")
        self.assertNotIn("transcribe", [c[0] for c in self.fake.calls])

    def test_focus_reaches_the_prompt_by_header_and_by_json(self):
        focus = json.dumps({"hovered": "tower-1", "locked": ["quadcopter"]})
        self.post(b"a", headers={"X-Focus": urllib.parse.quote(focus)})
        self.assertIn("pointing at tower-1", self.fake.prompt)
        self.assertIn("pinned these tags open: quadcopter", self.fake.prompt)
        self.post(json.dumps({"text": "hi", "focus": {"hovered": "boat"}}).encode(), "application/json")
        self.assertIn("pointing at boat", self.fake.prompt)
        self.post(b"a", headers={"X-Focus": "not json"})                            # garbage focus is ignored, not fatal
        self.assertNotIn("pointing at", self.fake.prompt)

    def test_bad_requests(self):
        self.assertEqual(self.post(b"", "audio/webm")[0], 400)
        status, body = self.post(b"hello", "text/plain")
        self.assertEqual((status, body["stage"]), (415, "request"))
        self.assertEqual(self.post(b"{not json", "application/json")[0], 400)
        self.assertEqual(self.post(b"x", path="/nope")[0], 404)
        with mock.patch.object(server, "MAX_BODY", 10):
            status, body = self.post(b"x" * 100)
        self.assertEqual((status, body["error"]), (413, "recording too large"))
        self.assertEqual(self.fake.calls, [])                                        # nothing was spent on any of them

    def test_silence_is_reported_not_sent_to_gpt(self):
        self.fake.transcript = "   "
        status, body = self.post(b"a")
        self.assertEqual((status, body["stage"]), (422, "transcribe"))
        self.assertEqual([c[0] for c in self.fake.calls], ["transcribe"])

    def test_each_failure_names_its_stage(self):
        self.fake.fail = "transcribe"
        status, body = self.post(b"a")
        self.assertEqual((status, body["stage"]), (502, "transcribe"))
        self.fake.fail = "key"
        status, body = self.post(b"a")
        self.assertEqual((status, body["stage"]), (503, "transcribe"))
        self.assertIn("OPENAI_API_KEY", body["error"])
        self.fake.fail = "answer"
        status, body = self.post(b"a")
        self.assertEqual((status, body["stage"], body["question"]), (502, "answer", "where is the boat"))

    def test_a_failed_voice_still_returns_the_answer(self):
        self.fake.fail = "speak"
        status, body = self.post(b"a")
        self.assertEqual(status, 200)
        self.assertEqual(body["answer"], "It is one kilometre north.")
        self.assertIsNone(body["audio_b64"])
        self.assertIn("voice down", body["speak_error"])

    def test_works_without_the_mission_running(self):
        self.start("http://127.0.0.1:1/positions")                                  # nothing there
        status, _ = self.post(b"a")
        self.assertEqual(status, 200)
        self.assertIn("unavailable", self.fake.prompt)

    def test_health_and_cors(self):
        with urllib.request.urlopen(self.base + "/health", timeout=5) as r:
            self.assertEqual(json.load(r)["feed"], True)
        self.start("http://127.0.0.1:1/positions")
        with urllib.request.urlopen(self.base + "/health", timeout=5) as r:
            self.assertEqual(json.load(r)["feed"], False)
        req = urllib.request.Request(self.base + "/ask", method="OPTIONS")
        with urllib.request.urlopen(req, timeout=5) as r:
            self.assertEqual((r.status, r.headers["Access-Control-Allow-Origin"]), (204, "*"))

    def test_demo_backends_need_no_keys_and_produce_playable_audio(self):
        self.start(self.feed_url, demo_backends())
        status, body = self.post(b"A" * 4096, headers={"X-Focus": urllib.parse.quote(json.dumps({"hovered": "boat"}))})
        self.assertEqual(status, 200)
        self.assertIn("demo mode", body["answer"])
        self.assertIn("pointing at boat", body["answer"])
        self.assertEqual(body["audio_mime"], "audio/wav")
        with wave.open(io.BytesIO(base64.b64decode(body["audio_b64"]))) as w:
            self.assertGreater(w.getnframes(), 1000)


class CoreTests(unittest.TestCase):
    def test_importing_needs_no_keys_or_audio_libraries(self):
        self.assertTrue(callable(core.transcribe))                                  # this module imported fine above

    def test_missing_keys_are_a_clear_error(self):
        with mock.patch.dict(os.environ, {}, clear=True), mock.patch.object(core, "_client", None):
            with self.assertRaises(core.MissingKey):
                core.transcribe(b"x", "audio/webm")
            with self.assertRaises(core.MissingKey):
                core.synthesize("hello")

    def fake_client(self):
        calls = {}

        def transcribe(**kw):
            calls["transcribe"] = kw
            return SimpleNamespace(text="  hello  ")

        def chat(**kw):
            calls["chat"] = kw
            return SimpleNamespace(choices=[SimpleNamespace(message=SimpleNamespace(content="  hi  "))])

        client = SimpleNamespace(audio=SimpleNamespace(transcriptions=SimpleNamespace(create=transcribe)),
                                 chat=SimpleNamespace(completions=SimpleNamespace(create=chat)))
        return client, calls

    def test_same_models_and_settings_as_voice_py(self):
        client, calls = self.fake_client()
        with mock.patch.object(core, "_client", client):
            self.assertEqual(core.transcribe(b"AUDIO", "audio/webm;codecs=opus"), "hello")
            self.assertEqual(core.answer("q", "system"), "hi")
        t = calls["transcribe"]
        self.assertEqual((t["model"], t["language"], t["file"]), ("whisper-1", "en", ("speech.webm", b"AUDIO", "audio/webm")))
        c = calls["chat"]
        self.assertEqual((c["model"], c["max_tokens"], c["temperature"]), ("gpt-4o", 150, 0.4))
        self.assertEqual([m["role"] for m in c["messages"]], ["system", "user"])

    def test_container_types_get_the_right_extension(self):
        client, calls = self.fake_client()
        with mock.patch.object(core, "_client", client):
            for mime, ext in (("audio/mp4", "mp4"), ("audio/ogg;codecs=opus", "ogg"), ("audio/wav", "wav"), ("audio/unknown", "webm")):
                core.transcribe(b"x", mime)
                self.assertEqual(calls["transcribe"]["file"][0], f"speech.{ext}")

    def test_elevenlabs_request_matches_voice_py(self):
        try:
            import requests  # noqa: F401
        except ImportError:
            self.skipTest("requests not installed")
        response = SimpleNamespace(status_code=200, content=b"MP3", text="")
        with mock.patch.dict(os.environ, {"ELEVENLABS_API_KEY": "k", "ELEVENLABS_VOICE_ID": "voice1"}), \
                mock.patch("requests.post", return_value=response) as post:
            self.assertEqual(core.synthesize("hello"), (b"MP3", "audio/mpeg"))
        args, kwargs = post.call_args
        self.assertEqual(args[0], "https://api.elevenlabs.io/v1/text-to-speech/voice1")
        self.assertEqual(kwargs["json"]["model_id"], "eleven_turbo_v2")
        self.assertEqual(kwargs["headers"]["xi-api-key"], "k")
        with mock.patch.dict(os.environ, {"ELEVENLABS_API_KEY": "k"}), \
                mock.patch("requests.post", return_value=SimpleNamespace(status_code=401, content=b"", text="bad key")):
            with self.assertRaisesRegex(RuntimeError, "401"):
                core.synthesize("hello")


if __name__ == "__main__":
    unittest.main()
