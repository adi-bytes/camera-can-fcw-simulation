# Camera-to-CAN Forward Collision Warning (simulation)

> **Simulation project — no physical CAN hardware or vehicle validation.** This repository links a monocular camera perception demo to a simulated CAN network. Do not use it to control a vehicle or make safety decisions.

## What it demonstrates

```
video / USB camera → YOLO vehicle detection + OpenCV lane overlay → camera CAN node
                                                                  │  UDP multicast (virtual CAN)
                                                                  ▼
                                             DBC decoder → ECU integrity checks → FCW status/log
```

The camera node transmits the closest detected road user at 20 Hz and a heartbeat at 2 Hz. The ECU decodes the DBC, validates an alive counter and CRC-8, watches for timeouts, and enters a safe state on faults. `--fault` modes deliberately exercise the safety path.

## One-day scope

| Area | Included | Deliberately not claimed |
|---|---|---|
| Perception | Pretrained YOLO (or a supplied custom `best.pt`), simple bounding-box distance estimate, visual lane overlay | Production object tracking, calibrated range sensor, safety validation |
| Network | DBC, CRC-8, rolling alive counter, watchdog, virtual transport | Physical CAN, ISO 26262 compliance |
| Faults | Drop, corrupt-CRC, stuck-counter | Bus-off / electrical faults |

## Quick start (Windows, macOS, Linux)

Python 3.10+ is recommended. `udp_multicast` is used by default so two independent processes can share a virtual bus on Windows; WSL users may alternatively select `socketcan` with a configured `vcan0` interface.

```bash
python -m venv .venv
.venv\Scripts\activate                 # Windows PowerShell
# source .venv/bin/activate              # macOS/Linux
pip install -e .
```

Terminal 1 — ECU:

```bash
python -m fcw.ecu_node --channel 239.255.60.60
```

Terminal 2 — deterministic camera simulation (works without a camera/model):

```bash
python -m fcw.camera_node --simulate --channel 239.255.60.60
```

To exercise failure handling, add one of:

```bash
--fault corrupt-crc --fault-after 3
--fault drop --fault-after 3
--fault stuck-counter --fault-after 3
```

For a video and YOLO weights (the default `yolov8n.pt` downloads on first use):

```bash
python -m fcw.camera_node --video path/to/dashcam.mp4 --weights yolov8n.pt --show
```

For Linux SocketCAN after `sudo modprobe vcan; sudo ip link add dev vcan0 type vcan; sudo ip link set up vcan0`:

```bash
python -m fcw.ecu_node --interface socketcan --channel vcan0
python -m fcw.camera_node --simulate --interface socketcan --channel vcan0
```

## Message contract

`dbc/fcw.dbc` is the source of truth. All multi-bit signals are Intel/little-endian.

| CAN ID | Message | Rate | Key signals |
|---|---|---:|---|
| `0x180` | `FCW_OBJECT` | 20 Hz | range (0.1 m), class, confidence, alive counter, CRC-8 |
| `0x181` | `CAMERA_HEARTBEAT` | 2 Hz | node state, alive counter |
| `0x280` | `FCW_STATUS` | 20 Hz | warning, safe state, reason, data age |

The CRC is SAE J1850 style (poly `0x1D`, init/xorout `0xFF`) over bytes 0–6 of `FCW_OBJECT`; byte 7 is the CRC field. The alive counter increments modulo 16. A missing object frame for 150 ms, CRC mismatch, or unexpected counter moves the ECU to a safe state. After a fault, three consecutive valid, in-sequence frames are required to leave that state. A warning is raised only for a valid `car`, `two_wheeler`, or `auto_rickshaw` within the configured threshold (default 8 m).

## Perception status

No self-collected dataset, custom training metrics, calibration results, demo GIF, or confusion matrix is committed in this repository. The default `yolov8n.pt` is a COCO model; it can identify cars, motorcycles, bicycles, and people, but does not provide dedicated auto-rickshaw or speed-breaker classes. A custom model can supply those labels through the same mapping layer.

The range estimate uses a configurable focal length and assumed target height. It is uncalibrated and must not be treated as a measured distance. The yellow lines are an OpenCV visual overlay; they are not an ego-lane classifier and the ECU does not use `LaneValid` to filter warnings.

To add genuine perception evidence, collect footage with consent and legal permission, label it, retain `data.yaml`, and record the exact training command and measured metrics. A starting command after exporting a YOLO dataset is:

```bash
yolo detect train model=yolov8n.pt data=dataset/data.yaml epochs=30 imgsz=640
```

## Tests

```bash
pytest -q
```

They cover DBC round-tripping and contract consistency, checksum rejection, counter recovery, and timeout-safe-state behavior. GitHub Actions runs the same suite on pushes and pull requests.

## Project layout

```
dbc/fcw.dbc             CAN message definition
src/fcw/protocol.py     DBC encoding/decoding and CRC
src/fcw/camera_node.py  video/simulation producer
src/fcw/ecu_node.py     watchdog and warning consumer
tests/                  deterministic protocol/ECU tests
```
