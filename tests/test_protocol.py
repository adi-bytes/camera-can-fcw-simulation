import time

from fcw.ecu_node import ForwardCollisionEcu
from fcw.protocol import CLASSES, database, decode_object, object_message


def frame(alive=0, distance=7, corrupt=False):
    return object_message(database(), distance_m=distance, class_id=CLASSES["car"],
                          confidence=.9, alive=alive, lane_valid=True, corrupt_crc=corrupt)


def test_dbc_object_round_trip_and_crc():
    decoded = decode_object(database(), frame(3, 7.4))
    assert decoded.alive == 3
    assert decoded.distance_m == 7.4
    assert decoded.class_id == CLASSES["car"]


def test_ecu_rejects_bad_crc_into_safe_state():
    ecu = ForwardCollisionEcu()
    decision = ecu.receive(frame(corrupt=True), 10.0)
    assert decision.safe and decision.reason == "crc" and not decision.warning


def test_ecu_rejects_counter_jump_into_safe_state():
    ecu = ForwardCollisionEcu()
    ecu.receive(frame(0), 10.0)
    decision = ecu.receive(frame(2), 10.05)
    assert decision.safe and decision.reason == "alive_counter"


def test_ecu_warns_then_times_out():
    ecu = ForwardCollisionEcu(timeout_s=.15)
    assert ecu.receive(frame(0, 5), 10.0).warning
    decision = ecu.tick(10.151)
    assert decision.safe and decision.reason == "timeout" and not decision.warning


def test_single_corrupt_frame_recovers_quickly_and_keeps_cause():
    ecu = ForwardCollisionEcu(recover_after=3)
    t = 10.0
    for a in range(3):
        ecu.receive(frame(a), t := t + .05)
    assert ecu.receive(frame(3, corrupt=True), t := t + .05).reason == "crc"
    # a non-object tick inside the timeout window must not overwrite the root cause
    assert ecu.tick(t + .01).reason == "crc"
    results = [ecu.receive(frame(a), t := t + .05) for a in range(4, 8)]
    assert results[0].safe and results[0].reason == "alive_counter"   # the gap is detected once
    assert not results[-1].safe                                       # then recovers, no 16-frame wait


def test_recovery_requires_consecutive_good_frames():
    ecu = ForwardCollisionEcu(recover_after=3)
    ecu.receive(frame(0), 10.0)
    ecu.receive(frame(5), 10.05)                 # jump -> safe, resync to 5
    assert ecu.receive(frame(6), 10.10).safe     # streak 1
    assert ecu.receive(frame(7), 10.15).safe     # streak 2
    assert not ecu.receive(frame(8), 10.20).safe # streak 3 -> leaves safe state


def test_threshold_boundary_and_class_filter():
    assert ForwardCollisionEcu().receive(frame(0, 7.9), 0).warning
    assert not ForwardCollisionEcu().receive(frame(0, 8.0), 0).warning
