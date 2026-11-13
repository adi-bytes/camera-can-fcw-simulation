"""Simulated ECU: validates FCW_OBJECT and publishes FCW_STATUS."""
from __future__ import annotations

import argparse
import logging
import time
from dataclasses import dataclass

from .protocol import CLASSES, OBJECT_ID, database, decode_object, open_bus, status_message

LOG = logging.getLogger("fcw.ecu")
REASON = {"none": 0, "timeout": 1, "crc": 2, "alive_counter": 3, "decode": 4}


@dataclass
class EcuDecision:
    warning: bool = False
    safe: bool = True
    reason: str = "timeout"
    age_ms: int = 65535


class ForwardCollisionEcu:
    def __init__(self, threshold_m: float = 8.0, timeout_s: float = 0.150):
        self.threshold_m, self.timeout_s = threshold_m, timeout_s
        self.last_alive: int | None = None
        self.last_valid_at: float | None = None
        self.decision = EcuDecision()

    def receive(self, message, now: float) -> EcuDecision:
        try:
            frame = decode_object(self.db, message)
        except ValueError as error:
            reason = "crc" if "CRC" in str(error) else "decode"
            self.decision = EcuDecision(safe=True, reason=reason)
            return self.decision
        if self.last_alive is not None and frame.alive != (self.last_alive + 1) % 16:
            self.decision = EcuDecision(safe=True, reason="alive_counter")
            return self.decision
        self.last_alive, self.last_valid_at = frame.alive, now
        relevant = frame.class_id in {CLASSES["car"], CLASSES["two_wheeler"], CLASSES["auto_rickshaw"]}
        self.decision = EcuDecision(warning=relevant and frame.distance_m < self.threshold_m,
                                    safe=False, reason="none", age_ms=0)
        return self.decision

    @property
    def db(self):
        if not hasattr(self, "_db"):
            self._db = database()
        return self._db

    def tick(self, now: float) -> EcuDecision:
        if self.last_valid_at is None or now - self.last_valid_at > self.timeout_s:
            age = 65535 if self.last_valid_at is None else round((now - self.last_valid_at) * 1000)
            self.decision = EcuDecision(safe=True, reason="timeout", age_ms=age)
        elif not self.decision.safe:
            self.decision.age_ms = round((now - self.last_valid_at) * 1000)
        return self.decision


def main() -> None:
    parser = argparse.ArgumentParser(description="Simulated FCW ECU (virtual CAN only)")
    parser.add_argument("--interface", default="udp_multicast")
    parser.add_argument("--channel", default="239.255.60.60")
    parser.add_argument("--threshold-m", type=float, default=8.0)
    parser.add_argument("--timeout-ms", type=float, default=150)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")
    ecu, db = ForwardCollisionEcu(args.threshold_m, args.timeout_ms / 1000), database()
    with open_bus(args.interface, args.channel) as bus:
        LOG.info("SIMULATION ONLY: ECU listening on %s/%s", args.interface, args.channel)
        prior = None
        while True:
            now = time.monotonic()
            message = bus.recv(timeout=0.03)
            if message and message.arbitration_id == OBJECT_ID:
                decision = ecu.receive(message, now)
            else:
                decision = ecu.tick(now)
            if (decision.warning, decision.safe, decision.reason) != prior:
                LOG.warning("status warning=%s safe=%s reason=%s age=%dms", decision.warning,
                            decision.safe, decision.reason, decision.age_ms)
                prior = (decision.warning, decision.safe, decision.reason)
            bus.send(status_message(db, warning=decision.warning, safe=decision.safe,
                                    reason=REASON[decision.reason], age_ms=decision.age_ms))


if __name__ == "__main__":
    main()
