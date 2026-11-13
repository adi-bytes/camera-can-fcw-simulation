from fcw.ecu_node import REASON
from fcw.protocol import CLASSES, HEARTBEAT_ID, OBJECT_ID, STATUS_ID, database


def test_python_message_ids_match_dbc():
    db = database()
    assert db.get_message_by_name("FCW_OBJECT").frame_id == OBJECT_ID
    assert db.get_message_by_name("CAMERA_HEARTBEAT").frame_id == HEARTBEAT_ID
    assert db.get_message_by_name("FCW_STATUS").frame_id == STATUS_ID


def test_python_choice_codes_match_dbc():
    db = database()
    object_choices = db.get_message_by_name("FCW_OBJECT").get_signal_by_name("ObjectClass").choices
    reason_choices = db.get_message_by_name("FCW_STATUS").get_signal_by_name("FaultReason").choices
    assert {str(name): code for name, code in CLASSES.items()} == {str(label): code for code, label in object_choices.items()}
    assert {str(name): code for name, code in REASON.items()} == {str(label): code for code, label in reason_choices.items()}
