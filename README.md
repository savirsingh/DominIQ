# DominIQ

> 4 drones, dropped with no idea where the enemy ship is. Within minutes, it's found — using just camera geometry, tracked by an algorithm, all while you watch in VR and clarify by voice.

Built at **Hack the North 2026**.

---

## What it does

DominIQ autonomously detects, locates, and tracks a moving vessel across a 6.5km Arctic environment using coordinated drones and AI. It fuses YOLO detections with real-time drone telemetry to compute the boat's GPS coordinates purely from camera geometry — no GPS on the target, no pre-known position, no human operator in the loop.

---

## How it works

**Four assets patrol the environment:**
- Quadcopter at 60m altitude, 1800m patrol radius
- Fixed-wing plane at 80m altitude, 2800m patrol radius
- Two stationary towers at opposite ends of the site

All four run a custom-trained YOLO model on their live camera feeds simultaneously. When any camera spots the vessel, the system computes its GPS coordinates using projective geometry: the detection bounding box center pixel is projected to ground using the camera FOV, drone altitude, heading, pitch, and physical mount angle. Both drones then converge and orbit the target at 400m standoff, continuously re-estimating its position as it moves.

On top of the flight stack, an operator can ask questions by voice — powered by OpenAI Whisper (speech-to-text), GPT-4o (reading live telemetry logs), and ElevenLabs (spoken responses).

---

## Stack

- **Flight control:** ArduPilot SITL over MAVLink (pymavlink)
- **Simulation:** Gazebo via gzweb
- **Object detection:** YOLOv8 (custom-trained on Arctic vessel imagery)
- **Position estimation:** Camera geometry + trig (no simulation cheating)
- **Voice assistant:** Whisper + GPT-4o + ElevenLabs
- **Visualization:** HTML5 canvas map with live WebSocket pose updates
- **VPN:** WireGuard (each teammate gets their own config)

---

## Setup

### 1. Connect to the SIM

```bash
# Ubuntu
sudo apt install wireguard
sudo cp SIM-8/arctic-sim-8-1.conf /etc/wireguard/arctic-sim-8-1.conf
sudo wg-quick up arctic-sim-8-1

# Or just run:
bash setup.sh
```

Each teammate needs a different config file (arctic-sim-8-1.conf through arctic-sim-8-10.conf).

### 2. Install dependencies

```bash
pip install -r requirements.txt
# For voice (mic support):
sudo apt install portaudio19-dev
pip install sounddevice
```

### 3. Open the SIM

Navigate to `http://10.99.7.1:8080` in Chrome.

---

## Usage

### Run the mission
```bash
python3 mission.py
```

Both drones take off, patrol in coordinated circles, and automatically detect and track the vessel using YOLO + camera geometry. Ctrl+C to land.

### Voice assistant (run alongside mission.py)
```bash
python3 voice.py
# With mic: press ENTER to speak, ENTER to stop
# Without mic: falls back to typed questions
```

### Manual flight tools
```bash
python3 fly.py status
python3 fly.py takeoff quadcopter 50
python3 fly.py goto quadcopter 71.990 -94.830 80
python3 fly.py land quadcopter
```

### Live map
Open `map.html` in a browser while the mission is running for a real-time top-down view of all assets and the estimated boat position.

---

## Architecture

```
mission.py
├── fly_copter()       quadcopter: patrol → detect → orbit
├── fly_plane()        fixed-wing: patrol → detect → orbit
├── run_tower()        tower-1/2: YOLO scan, flag detections
├── run_scanner()      YOLO inference thread (non-blocking)
└── bbox_to_gps()      camera geometry: pixel → GPS coords

voice.py
├── record_until_enter()   mic input
├── transcribe()           Whisper STT
├── ask_gpt()              GPT-4o with live mission context
└── speak()                ElevenLabs TTS
```

---

## Team

- **Savir Singh**
- **Émilien Lavallée**
- **Karan Gupta**
- **Helen Huang**
