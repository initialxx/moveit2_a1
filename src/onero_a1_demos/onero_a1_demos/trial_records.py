"""ROS-independent identity fields for repeatable motion reports."""

import hashlib
import json

BRIDGE_IDENTITY = ("model", "mounting", "can_channel", "simulate", "with_gripper", "gripper_model",
                   "gravity_compensation", "control_rate",
                   "gripper_closed_sdk_opening", "gripper_open_sdk_opening")


def fingerprint(value):
    payload = json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(payload.encode()).hexdigest()
