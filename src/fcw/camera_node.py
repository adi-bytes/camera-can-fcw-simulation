"""Camera producer for the FCW virtual-CAN simulation."""
from __future__ import annotations

import argparse
import logging
import time
from dataclasses import dataclass
from itertools import cycle

from .protocol import CLASSES, database, heartbeat_message, object_message, open_bus

LOG = logging.getLogger("fcw.camera")
LABEL_TO_FCW = {
    "car": "car", "motorcycle": "two_wheeler", "motorbike": "two_wheeler",
    "bicycle": "two_wheeler", "bike": "two_wheeler", "auto_rickshaw": "auto_rickshaw",
    "auto-rickshaw": "auto_rickshaw", "rickshaw": "auto_rickshaw", "pedestrian": "pedestrian",
    "person": "pedestrian", "speed_breaker": "speed_breaker",
}


@dataclass
class Detection:
    distance_m: float
    label: str
    confidence: float
    object_id: int = 0
    lane_valid: bool = False


def estimate_distance_m(box_height_px: float, focal_length_px: float, target_height_m: float = 1.5) -> float:
    """Monocular approximation: distance = known target height * focal length / pixel height.

    Calibrate focal_length_px and target_height_m for the actual camera; this is a demo,
    not a validated range measurement.
    """
    return 999.0 if box_height_px <= 0 else target_height_m * focal_length_px / box_height_px


def lane_overlay(frame):
    import cv2
    import numpy as np
    height, width = frame.shape[:2]
    roi = frame[int(height * 0.55):, :]
    edges = cv2.Canny(cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY), 50, 150)
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 35, minLineLength=35, maxLineGap=25)
    overlay, valid = frame.copy(), False
    if lines is not None:
        for x1, y1, x2, y2 in lines[:, 0]:
            if abs(x2 - x1) < 5:
                continue
            cv2.line(overlay, (x1, y1 + int(height * .55)), (x2, y2 + int(height * .55)), (0, 255, 255), 3)
            valid = True
    return overlay, valid


def video_detections(video: str, weights: str, focal_length_px: float, show: bool):
    import cv2
    from ultralytics import YOLO
    model, cap = YOLO(weights), cv2.VideoCapture(video)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open video: {video}")
    while True:
        ok, frame = cap.read()
        if not ok:
            return
        overlay, lane_valid = lane_overlay(frame)
        candidates = []
        for result in model(frame, verbose=False, stream=True):
            for box in result.boxes:
                source_label = str(result.names[int(box.cls[0])]).lower().replace(" ", "_")
                if source_label not in LABEL_TO_FCW:
                    continue
                x1, y1, x2, y2 = (float(v) for v in box.xyxy[0])
                distance = estimate_distance_m(y2 - y1, focal_length_px)
                label, confidence = LABEL_TO_FCW[source_label], float(box.conf[0])
                candidates.append(Detection(distance, label, confidence, int(box.id[0]) if box.id is not None else 0, lane_valid))
                cv2.rectangle(overlay, (int(x1), int(y1)), (int(x2), int(y2)), (0, 220, 0), 2)
                cv2.putText(overlay, f"{label} {distance:.1f}m", (int(x1), max(25, int(y1)-8)), cv2.FONT_HERSHEY_SIMPLEX, .65, (0,220,0), 2)
        if show:
            cv2.imshow("FCW simulation — ESC to exit", overlay)
            if cv2.waitKey(1) == 27:
                return
        yield min(candidates, key=lambda item: item.distance_m) if candidates else Detection(409.5, "unknown", 0, lane_valid=lane_valid)


def simulated_detections():
    """Repeatable approach/recede pattern for a hardware-free demonstration."""
    for distance in cycle([20, 16, 12, 9, 7, 5, 7, 10, 15]):
        yield Detection(float(distance), "car", .92, 1, True)


def main() -> None:
    parser = argparse.ArgumentParser(description="Camera-to-CAN FCW producer (simulation only)")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--simulate", action="store_true", help="send deterministic synthetic detections")
    source.add_argument("--video", help="dashcam/video input path")
    parser.add_argument("--weights", default="yolov8n.pt")
    parser.add_argument("--focal-length-px", type=float, default=900)
    parser.add_argument("--show", action="store_true")
    parser.add_argument("--interface", default="udp_multicast")
    parser.add_argument("--channel", default="239.255.60.60")
    parser.add_argument("--rate-hz", type=float, default=20)
    parser.add_argument("--fault", choices=["none", "drop", "corrupt-crc", "stuck-counter"], default="none")
    parser.add_argument("--fault-after", type=float, default=0, help="seconds before injection")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")
    detections = simulated_detections() if args.simulate else video_detections(args.video, args.weights, args.focal_length_px, args.show)
    db, alive, heartbeat_alive, started = database(), 0, 0, time.monotonic()
    next_heartbeat = started
    period = 1 / args.rate_hz
    with open_bus(args.interface, args.channel) as bus:
        LOG.info("SIMULATION ONLY: camera node sending at %.1f Hz", args.rate_hz)
        for detection in detections:
            tick = time.monotonic()
            if tick >= next_heartbeat:
                bus.send(heartbeat_message(db, state=1, alive=heartbeat_alive))
                heartbeat_alive = (heartbeat_alive + 1) % 16
                next_heartbeat = max(next_heartbeat + .5, tick)
            fault_active = args.fault != "none" and tick - started >= args.fault_after
            if fault_active and args.fault == "drop":
                LOG.warning("fault injected: dropping FCW_OBJECT")
            else:
                frame_alive = alive if not (fault_active and args.fault == "stuck-counter") else (alive - 1) % 16
                msg = object_message(db, distance_m=detection.distance_m, class_id=CLASSES[detection.label],
                                     confidence=detection.confidence, alive=frame_alive,
                                     lane_valid=detection.lane_valid, object_id=detection.object_id,
                                     corrupt_crc=fault_active and args.fault == "corrupt-crc")
                bus.send(msg)
            alive = (alive + 1) % 16
            remaining = period - (time.monotonic() - tick)
            if remaining > 0:
                time.sleep(remaining)


if __name__ == "__main__":
    main()
