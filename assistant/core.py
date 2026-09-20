"""The three network steps of the voice pipeline, as plain functions: Whisper, GPT-4o, ElevenLabs.

The same models and settings as voice.py. Nothing here runs at import time and no key is read until it is
needed, so importing this module never fails for a missing key or a missing audio library.
"""
from __future__ import annotations

import os
import threading
from dataclasses import dataclass
from typing import Callable, Optional, Tuple

ELEVENLABS_MODEL = "eleven_turbo_v2"
DEFAULT_VOICE_ID = "EXAVITQu4vr4xnSDxMaL"  # same default as voice.py

# What the browser's MediaRecorder produces -> a file extension Whisper recognizes.
_EXTENSIONS = {"audio/webm": "webm", "audio/ogg": "ogg", "audio/mp4": "mp4", "audio/mpeg": "mp3",
               "audio/wav": "wav", "audio/x-wav": "wav", "audio/m4a": "m4a"}


class MissingKey(RuntimeError):
    pass


@dataclass
class Backends:
    """The three steps, swappable so tests and --demo need no keys."""
    transcribe: Callable[[bytes, str], str]                       # (audio, mime) -> text
    answer: Callable[[str, str], str]                             # (question, system_prompt) -> text
    synthesize: Callable[[str], Optional[Tuple[bytes, str]]]      # text -> (audio, mime)


_client = None
_client_lock = threading.Lock()


def _openai():
    global _client
    with _client_lock:
        if _client is None:
            key = os.environ.get("OPENAI_API_KEY")
            if not key:
                raise MissingKey("OPENAI_API_KEY is not set")
            from openai import OpenAI
            _client = OpenAI(api_key=key, timeout=30, max_retries=1)
        return _client


def transcribe(audio: bytes, mime: str) -> str:
    base = mime.split(";")[0].strip().lower()
    result = _openai().audio.transcriptions.create(
        model="whisper-1",
        file=(f"speech.{_EXTENSIONS.get(base, 'webm')}", audio, base or "audio/webm"),
        language="en",
    )
    return result.text.strip()


def answer(question: str, system_prompt: str) -> str:
    response = _openai().chat.completions.create(
        model="gpt-4o",
        messages=[{"role": "system", "content": system_prompt}, {"role": "user", "content": question}],
        max_tokens=150,
        temperature=0.4,
    )
    return response.choices[0].message.content.strip()


def synthesize(text: str) -> Optional[Tuple[bytes, str]]:
    import requests

    key = os.environ.get("ELEVENLABS_API_KEY")
    if not key:
        raise MissingKey("ELEVENLABS_API_KEY is not set")
    voice = os.environ.get("ELEVENLABS_VOICE_ID", DEFAULT_VOICE_ID)
    r = requests.post(
        f"https://api.elevenlabs.io/v1/text-to-speech/{voice}",
        headers={"xi-api-key": key, "Content-Type": "application/json"},
        json={"text": text, "model_id": ELEVENLABS_MODEL, "voice_settings": {"stability": 0.5, "similarity_boost": 0.75}},
        timeout=20,
    )
    if r.status_code != 200:
        raise RuntimeError(f"ElevenLabs {r.status_code}: {r.text[:120]}")
    return r.content, "audio/mpeg"


def real_backends() -> Backends:
    return Backends(transcribe=transcribe, answer=answer, synthesize=synthesize)
