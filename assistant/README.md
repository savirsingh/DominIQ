# Voice assistant for the WebXR view

Hold a button on the Quest controller, ask about the mission out loud, and get a spoken answer plus a
caption. Same pipeline as `voice.py` (mic -> Whisper -> GPT-4o -> ElevenLabs), but run as a small service so
the headset never holds an API key.

```
Quest: hold B, speak, release ──► POST /ask (audio) ──► vite dev server proxy
                                                              │
assistant/server.py (this folder, its own process)  ◄─────────┘
   Whisper -> GPT-4o (with the mission's live state) -> ElevenLabs
                                                              │
Quest: caption on the card in view + the voice  ◄─────────────┘
```

It is a separate process from `mission.py` on purpose: a slow or failing API call cannot touch the flight
and detection threads. It only *reads* the mission's position feed; it never talks MAVLink.

## Run

```bash
pip install -r assistant/requirements.txt          # openai, requests, python-dotenv
# .env in the repo root (already gitignored):
#   OPENAI_API_KEY=...   ELEVENLABS_API_KEY=...   ELEVENLABS_VOICE_ID=... (optional)

python3 mission.py --sim local --webxr-feed        # terminal 1: the mission, serving live positions
python3 assistant/server.py                        # terminal 2: the assistant on 127.0.0.1:8782
cd webxr && npm run dev                            # terminal 3: the headset app
```

No keys yet? `python3 assistant/server.py --demo` answers with canned text and a beep, so the whole
headset loop (recording, sending, caption, audio playback) can be tried at no cost.

## Use

Hold **B** on the right controller (or **Y** on the left), speak, release. Press again while it is talking to
interrupt it. The card in view shows *listening*, *thinking*, then the answer with what it heard in quotes.
The buttons are `TALK_BUTTON_RIGHT` / `TALK_BUTTON_LEFT` at the top of `webxr/src/main.ts`.

It knows: every asset's position and altitude, the boat's estimated position, and the distance and bearing
from each asset to the boat and to each other (computed here, since language models are poor at lat/lon
arithmetic). It also knows which asset you are pointing at and which tags you have pinned open, so
"what is that one doing?" works. If a mission log exists it adds recent detections, motion and errors:

```bash
python3 mission.py --sim local --webxr-feed 2>&1 | tee /tmp/mission_out.log
```

## Notes

- **Microphone and audio need a click first.** The app asks on the first click anywhere on the page, so click
  the page (or *Enter AR*) once before the session starts. If the mic prompt is denied the card says so.
- **Local only.** `/ask` spends API credits, so it binds to 127.0.0.1. The headset reaches it through the
  dev server, which is already on your network. `--bind` changes that; don't do it on a shared network.
- **Latency** is three network calls in a row, usually 3 to 5 seconds. The server prints each step's time.
- **If speech synthesis fails** the answer still appears as text.
- Audio leaves your machine for OpenAI and ElevenLabs.

## Tests

```bash
python3 -m unittest discover -s assistant/tests -v     # no keys or network needed
```

## Files

`server.py` HTTP service and CLI · `core.py` the three API calls (same models and settings as `voice.py`) ·
`context.py` mission state -> prompt text · `demo.py` key-less stand-ins
