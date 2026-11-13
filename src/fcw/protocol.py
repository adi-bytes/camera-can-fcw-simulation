"""DBC-backed wire protocol helpers for the FCW simulation."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import can
import cantools

OBJECT_ID = 0x180
HEARTBEAT_ID = 0x181
STATUS_ID = 0x280
CLASSES = {"unknown": 0, "car": 1, "two_wheeler": 2, "auto_rickshaw": 3,
           "pedestrian": 4, "speed_breaker": 5}


def crc8_j1850(payload: bytes) -> int:
    """CRC-8/SAE-J1850: poly 0x1D, init/xorout 0xFF, no reflection."""
    crc = 0xFF
    for value in payload:
        crc ^= value
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc ^ 0xFF


def database() -> cantools.database.Database:
    return cantools.database.load_file(Path(__file__).parents[2] / "dbc" / "fcw.dbc")


def open_bus(interface: str, channel: str) -> can.BusABC:
    """Open a shared virtual transport. udp_multicast supports separate processes."""
    kwargs: dict[str, Any] = {"interface": interface, "channel": channel}
    if interface == "udp_multicast":
        kwargs["fd"] = False
    return can.Bus(**kwargs)


def object_message(db: cantools.database.Database, *, distance_m: float, class_id: int,
                   confidence: float, alive: int, lane_valid: bool, object_id: int = 0,
                   corrupt_crc: bool = False) -> can.Message:
    msg = db.get_message_by_name("FCW_OBJECT")
    raw = bytearray(msg.encode({
        "Distance_m": max(0, min(409.5, distance_m)), "ObjectClass": class_id,
        "Confidence_pct": round(max(0, min(100, confidence * 100))),
        "AliveCounter": alive % 16, "LaneValid": int(lane_valid),
        "ObjectId": object_id % 256, "CRC8": 0,
    }))
    raw[7] = crc8_j1850(raw[:7]) ^ (0xFF if corrupt_crc else 0)
    return can.Message(arbitration_id=OBJECT_ID, data=raw, is_extended_id=False)


def status_message(db: cantools.database.Database, *, warning: bool, safe: bool,
                   reason: int, age_ms: int) -> can.Message:
    raw = db.get_message_by_name("FCW_STATUS").encode({
        "WarningActive": int(warning), "SafeState": int(safe), "FaultReason": reason,
        "DetectionAge_ms": min(65535, max(0, age_ms)),
    })
    return can.Message(arbitration_id=STATUS_ID, data=raw, is_extended_id=False)


@dataclass(frozen=True)
class ObjectFrame:
    distance_m: float
    class_id: int
    confidence: int
    alive: int
    lane_valid: bool
    object_id: int


def decode_object(db: cantools.database.Database, message: can.Message) -> ObjectFrame:
    if message.arbitration_id != OBJECT_ID or len(message.data) != 8:
        raise ValueError("not an 8-byte FCW_OBJECT frame")
    if crc8_j1850(bytes(message.data[:7])) != message.data[7]:
        raise ValueError("CRC mismatch")
    # Choices in the DBC are useful to CAN tooling, but ECU logic needs scalars.
    data = db.get_message_by_name("FCW_OBJECT").decode(message.data, decode_choices=False)
    return ObjectFrame(float(data["Distance_m"]), int(data["ObjectClass"]),
                       int(data["Confidence_pct"]), int(data["AliveCounter"]),
                       bool(data["LaneValid"]), int(data["ObjectId"]))
