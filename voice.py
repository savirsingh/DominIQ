#!/usr/bin/env python3
"""
Voice assistant for Arctic SIM-8 mission.

Reads mission state from /tmp/mission_out.log and the SIM API,
lets you ask questions by voice, and answers out loud.

Usage:  python3 voice.py
        python3 voice.py --text "where is the boat"   # text mode (no mic)

Press ENTER to start recording, ENTER again to stop and get an answer.
"""

import os
import sys
import json
import time
import tempfile
import threading
import argparse
import requests
import subprocess
import io

import numpy as np
import scipy.io.wavfile as wav
from openai import OpenAI

try:
    import sounddevice as sd
    _mic_available = True
except OSError:
    sd = None
    _mic_available = False

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

from dotenv import load_dotenv
load_dotenv()

OPENAI_API_KEY = os.environ["OPENAI_API_KEY"]
ELEVENLABS_API_KEY = os.environ["ELEVENLABS_API_KEY"]
ELEVENLABS_VOICE_ID = os.getenv("ELEVENLABS_VOICE_ID", "EXAVITQu4vr4xnSDxMaL")
SIM_API = "http://10.99.7.1:8090"
LOG_FILE = "/tmp/mission_out.log"
SAMPLE_RATE = 16000

client = OpenAI(api_key=OPENAI_API_KEY)

# ---------------------------------------------------------------------------
# Mission context builder
# ---------------------------------------------------------------------------

def get_mission_state():
    """Read the latest mission state from the log and SIM API."""
    state = {}

    # Read last 60 lines of mission log
    try:
        result = subprocess.run(["tail", "-60", LOG_FILE],
                                capture_output=True, text=True)
        log_tail = result.stdout
    except Exception:
        log_tail = ""

    # Parse latest STATUS block from log
    lines = log_tail.splitlines()
    asset_lines = []
    boat_line = ""
    in_status = False
    for line in lines:
        if "--- STATUS ---" in line:
            in_status = True
            asset_lines = []
            boat_line = ""
        elif in_status:
            if line.strip() == "":
                in_status = False
            elif "BOAT(" in line:
                boat_line = line.strip()
            elif any(n in line for n in ["quadcopter", "fixed-wing", "tower"]):
                asset_lines.append(line.strip())

    state["assets"] = asset_lines
    state["boat"] = boat_line

    # Check for detection events
    detections = [l for l in lines if "BOAT DETECTED" in l or "BOAT SPOTTED" in l]
    state["detections"] = detections[-3:] if detections else []

    # Check for errors
    errors = [l for l in lines if "ARM FAILED" in l or "error:" in l.lower()]
    state["errors"] = errors[-3:] if errors else []

    # Query SIM API for asset status
    try:
        r = requests.get(f"{SIM_API}/api/assets", timeout=3)
        assets = r.json().get("assets", [])
        state["mavlink"] = {a["name"]: a["mavlink"] for a in assets}
    except Exception:
        state["mavlink"] = {}

    try:
        r = requests.get(f"{SIM_API}/api/status", timeout=3)
        state["sim_status"] = r.json()
    except Exception:
        state["sim_status"] = {}

    return state


def build_system_prompt(state):
    ctx_parts = [
        "You are the AI operator assistant for the Arctic SIM-8 drone mission.",
        "Your job is to answer the operator's questions about what's happening in the simulation.",
        "Be concise and direct — this is a voice interface, so keep answers to 2-3 sentences max.",
        "Use natural spoken language, no markdown, no bullet points.",
        "",
        "=== CURRENT MISSION STATE ===",
    ]

    if state["assets"]:
        ctx_parts.append("Asset positions:")
        for a in state["assets"]:
            ctx_parts.append(f"  {a}")
    else:
        ctx_parts.append("Asset positions: no data yet (drones may still be taking off)")

    if state["boat"]:
        ctx_parts.append(f"Boat estimate: {state['boat']}")
    else:
        ctx_parts.append("Boat: not yet detected")

    if state["detections"]:
        ctx_parts.append("Recent detections:")
        for d in state["detections"]:
            ctx_parts.append(f"  {d}")

    if state["mavlink"]:
        online = [k for k, v in state["mavlink"].items() if v]
        offline = [k for k, v in state["mavlink"].items() if not v]
        if online:
            ctx_parts.append(f"MAVLink online: {', '.join(online)}")
        if offline:
            ctx_parts.append(f"MAVLink offline: {', '.join(offline)}")

    if state["errors"]:
        ctx_parts.append("Recent errors:")
        for e in state["errors"]:
            ctx_parts.append(f"  {e}")

    if state["sim_status"]:
        ctx_parts.append(f"SIM status: {state['sim_status'].get('state', 'unknown')}")

    return "\n".join(ctx_parts)

# ---------------------------------------------------------------------------
# Audio recording
# ---------------------------------------------------------------------------

_recording = False
_frames = []

def record_until_enter():
    global _recording, _frames
    _frames = []
    _recording = True

    def callback(indata, frames, t, status):
        if _recording:
            _frames.append(indata.copy())

    print("🎤 Recording... press ENTER to stop")
    with sd.InputStream(samplerate=SAMPLE_RATE, channels=1,
                        dtype='int16', callback=callback):
        input()

    _recording = False
    if not _frames:
        return None

    audio = np.concatenate(_frames, axis=0)
    tmp = tempfile.NamedTemporaryFile(suffix=".wav", delete=False)
    wav.write(tmp.name, SAMPLE_RATE, audio)
    return tmp.name

# ---------------------------------------------------------------------------
# Transcription (Whisper)
# ---------------------------------------------------------------------------

def transcribe(audio_path):
    with open(audio_path, "rb") as f:
        result = client.audio.transcriptions.create(
            model="whisper-1",
            file=f,
            language="en"
        )
    os.unlink(audio_path)
    return result.text.strip()

# ---------------------------------------------------------------------------
# Answer (GPT-4)
# ---------------------------------------------------------------------------

def ask_gpt(question, system_prompt):
    response = client.chat.completions.create(
        model="gpt-4o",
        messages=[
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": question},
        ],
        max_tokens=150,
        temperature=0.4,
    )
    return response.choices[0].message.content.strip()

# ---------------------------------------------------------------------------
# Text-to-speech (ElevenLabs)
# ---------------------------------------------------------------------------

def speak(text):
    url = f"https://api.elevenlabs.io/v1/text-to-speech/{ELEVENLABS_VOICE_ID}"
    headers = {
        "xi-api-key": ELEVENLABS_API_KEY,
        "Content-Type": "application/json",
    }
    payload = {
        "text": text,
        "model_id": "eleven_turbo_v2",
        "voice_settings": {"stability": 0.5, "similarity_boost": 0.75},
    }
    r = requests.post(url, headers=headers, json=payload, timeout=15)
    if r.status_code != 200:
        print(f"ElevenLabs error: {r.status_code} {r.text[:200]}")
        print(f"[Answer]: {text}")
        return

    # Write to temp file and play with pygame
    tmp = tempfile.NamedTemporaryFile(suffix=".mp3", delete=False)
    tmp.write(r.content)
    tmp.close()

    try:
        import pygame
        pygame.mixer.init()
        pygame.mixer.music.load(tmp.name)
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():
            time.sleep(0.1)
        pygame.mixer.quit()
    except Exception:
        # Fallback: use system player
        subprocess.run(["mpg123", "-q", tmp.name], capture_output=True)
    finally:
        os.unlink(tmp.name)

# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

def answer_question(question):
    print(f"\n❓ Question: {question}")
    print("🤔 Thinking...")

    state = get_mission_state()
    system_prompt = build_system_prompt(state)
    answer = ask_gpt(question, system_prompt)

    print(f"💬 Answer: {answer}\n")
    speak(answer)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--text", help="Ask a question by text instead of voice")
    args = parser.parse_args()

    print("=" * 55)
    print("ARCTIC SIM-8 — VOICE ASSISTANT")
    print("=" * 55)
    print("Ask anything about the mission state.")
    print("Examples: 'Where is the boat?'")
    print("          'What altitude is the quadcopter at?'")
    print("          'Has the boat been detected?'")
    print("          'What's the status of the mission?'")
    print()

    if args.text:
        answer_question(args.text)
        return

    if not _mic_available:
        print("Mic not available (install portaudio: sudo apt install portaudio19-dev)")
        print("Running in text mode instead.\n")
        try:
            while True:
                q = input("[ Type your question (Ctrl+C to quit) ]: ").strip()
                if q:
                    answer_question(q)
        except KeyboardInterrupt:
            print("\nBye!")
        return

    print("Press ENTER to start speaking, ENTER again to stop. Ctrl+C to quit.\n")

    try:
        while True:
            input("[ Press ENTER to ask a question ]")
            audio_path = record_until_enter()
            if not audio_path:
                print("No audio recorded, try again.")
                continue

            print("📝 Transcribing...")
            question = transcribe(audio_path)
            if not question:
                print("Couldn't understand that, try again.")
                continue

            answer_question(question)

    except KeyboardInterrupt:
        print("\nBye!")


if __name__ == "__main__":
    main()
