"""Stand-in backends for `server.py --demo`: exercise the whole headset loop with no API keys and no cost."""
from __future__ import annotations

import io
import math
import re
import struct
import wave
from typing import Optional, Tuple

from core import Backends


def _beep(seconds: float = 0.35, hz: float = 660.0, rate: int = 16000) -> bytes:
    """A short tone, so the headset's audio playback can be checked without a text-to-speech service."""
    out = io.BytesIO()
    with wave.open(out, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(rate)
        n = int(seconds * rate)
        w.writeframes(b"".join(
            struct.pack("<h", int(9000 * math.sin(2 * math.pi * hz * i / rate) * min(1, i / 400, (n - i) / 400)))
            for i in range(n)))
    return out.getvalue()


def demo_backends() -> Backends:
    def transcribe(audio: bytes, mime: str) -> str:
        return f"(demo) I heard {len(audio) // 1024} kilobytes of audio."

    def answer(question: str, system_prompt: str) -> str:
        n = len(re.findall(r"latitude", system_prompt))
        focus = re.search(r"pointing at ([\w-]+)", system_prompt)
        pointing = f" You are pointing at {focus.group(1)}." if focus else ""
        return f"This is demo mode, so there is no language model. The feed shows {n} positions.{pointing}"

    def synthesize(text: str) -> Optional[Tuple[bytes, str]]:
        return _beep(), "audio/wav"

    return Backends(transcribe=transcribe, answer=answer, synthesize=synthesize)
