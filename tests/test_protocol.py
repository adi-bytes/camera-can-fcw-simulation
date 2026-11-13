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
