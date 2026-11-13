"""Simulated ECU: validates FCW_OBJECT and publishes FCW_STATUS."""
from __future__ import annotations

import argparse
import logging
import time
from dataclasses import dataclass

from .protocol import (CLASSES, OBJECT_ID, CrcError, FrameError, database, decode_object,
                       open_bus, status_message)

LOG = logging.getLogger("fcw.ecu")
REASON = {"none": 0, "timeout": 1, "crc": 2, "alive_counter": 3, "decode": 4}


@dataclass
class EcuDecision:
    warning: bool = False
    safe: bool = True
    reason: str = "timeout"
    age_ms: int = 65535


class ForwardCollisionEcu:
    """Validates FCW_OBJECT frames. Leaves the safe state only after `recover_after`
    consecutive valid, in-sequence frames (debounced recovery)."""

    def __init__(self, threshold_m: float = 8.0, timeout_s: float = 0.150, recover_after: int = 3):
        self.threshold_m, self.timeout_s, self.recover_after = threshold_m, timeout_s, recover_after
        self.last_alive: int | None = None
        self.last_valid_at: float | None = None
        self.good_streak = 0
        self.faulted = False                     # True after a fault/outage; boot is not a fault
        self.decision = EcuDecision()

    def _fault(self, reason: str, age_ms: int = 65535) -> EcuDecision:
        self.good_streak, self.faulted = 0, True
        self.decision = EcuDecision(safe=True, reason=reason, age_ms=age_ms)
        return self.decision

    def receive(self, message, now: float) -> EcuDecision:
        try:
            frame = decode_object(self.db, message)
        except CrcError:
            return self._fault("crc")
        except FrameError:
            return self._fault("decode")
        in_sequence = self.last_alive is None or frame.alive == (self.last_alive + 1) % 16
        self.last_alive = frame.alive            # always resynchronise to the sender
        if not in_sequence:
            return self._fault("alive_counter")
        self.last_valid_at = now
        self.good_streak += 1
        if self.faulted and self.good_streak < self.recover_after:
            return self.decision                 # still recovering: stay safe, keep the cause
        self.faulted = False
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
        stale = self.last_valid_at is None or now - self.last_valid_at > self.timeout_s
        if stale:
            if not (self.decision.safe and self.decision.reason not in ("none", "timeout")):
                age = 65535 if self.last_valid_at is None else round((now - self.last_valid_at) * 1000)
                self.good_streak = 0
                self.faulted = self.last_valid_at is not None   # outage after valid data
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
    status_period = 0.05                         # FCW_STATUS cyclic transmit, 20 Hz
    with open_bus(args.interface, args.channel) as bus:
        LOG.info("SIMULATION ONLY: ECU listening on %s/%s", args.interface, args.channel)
        prior, next_tx = None, time.monotonic()
        while True:
            message = bus.recv(timeout=max(0.0, min(0.03, next_tx - time.monotonic())))
            now = time.monotonic()
            # Only FCW_OBJECT frames drive the ECU; everything else (including our own
            # FCW_STATUS echoed back by the virtual bus) just advances the watchdog.
            if message is not None and message.arbitration_id == OBJECT_ID:
                decision = ecu.receive(message, now)
            else:
                decision = ecu.tick(now)
            if (decision.warning, decision.safe, decision.reason) != prior:
                LOG.warning("status warning=%s safe=%s reason=%s age=%dms", decision.warning,
                            decision.safe, decision.reason, decision.age_ms)
                prior = (decision.warning, decision.safe, decision.reason)
            if now >= next_tx:
                bus.send(status_message(db, warning=decision.warning, safe=decision.safe,
                                        reason=REASON[decision.reason], age_ms=decision.age_ms))
                next_tx = max(next_tx + status_period, now)


if __name__ == "__main__":
    main()
