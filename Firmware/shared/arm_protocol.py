"""
arm_protocol.py – Shared message definitions for the pi.FO arm protocol.

The hover flight controller (Hover MCU) sends arm commands to the arm
controller (Arm MCU) over a UART link using a simple binary framing:

    [ 0xAF | cmd_id (1 B) | payload_len (1 B) | payload (N B) | checksum (1 B) ]

Checksum = XOR of all bytes from cmd_id through the last payload byte.
"""

FRAME_START = 0xAF

# Command IDs
CMD_SET_JOINTS = 0x01   # Set all 5 joint angles (+ gripper) in one packet
CMD_STOP       = 0x02   # Stop all servos (hold current position)
CMD_HOME       = 0x03   # Move arm to home/stow position
CMD_GRIPPER    = 0x04   # Open or close gripper only

# Joint indices within a CMD_SET_JOINTS payload
JOINT_BASE         = 0
JOINT_SHOULDER     = 1
JOINT_ELBOW        = 2
JOINT_WRIST_PITCH  = 3
JOINT_WRIST_ROLL   = 4
JOINT_GRIPPER      = 5  # end-effector (not a DOF, but included for convenience)

NUM_JOINTS = 5   # proper DOF count (excludes gripper)


def _checksum(data: bytes) -> int:
    """Return XOR checksum of *data*."""
    result = 0
    for b in data:
        result ^= b
    return result


def encode_set_joints(angles_deg: list, gripper_pct: float = 0.0) -> bytes:
    """
    Build a CMD_SET_JOINTS frame.

    Parameters
    ----------
    angles_deg : list[float]
        Five joint angles in degrees: [base, shoulder, elbow, wrist_pitch,
        wrist_roll]. Each value is clamped to [0, 180].
    gripper_pct : float
        Gripper open percentage 0.0 (closed) – 1.0 (fully open).

    Returns
    -------
    bytes
        Complete framed packet ready to transmit over UART.
    """
    if len(angles_deg) != NUM_JOINTS:
        raise ValueError(f"Expected {NUM_JOINTS} joint angles, got {len(angles_deg)}")

    # Encode each angle as a single byte (0–180°)
    payload = bytes([max(0, min(180, int(a))) for a in angles_deg])
    # Encode gripper as a single byte (0–100 %)
    payload += bytes([max(0, min(100, int(gripper_pct * 100)))])

    cmd = bytes([CMD_SET_JOINTS, len(payload)]) + payload
    return bytes([FRAME_START]) + cmd + bytes([_checksum(cmd)])


def encode_stop() -> bytes:
    """Build a CMD_STOP frame."""
    cmd = bytes([CMD_STOP, 0x00])
    return bytes([FRAME_START]) + cmd + bytes([_checksum(cmd)])


def encode_home() -> bytes:
    """Build a CMD_HOME frame."""
    cmd = bytes([CMD_HOME, 0x00])
    return bytes([FRAME_START]) + cmd + bytes([_checksum(cmd)])


def encode_gripper(open_pct: float) -> bytes:
    """
    Build a CMD_GRIPPER frame.

    Parameters
    ----------
    open_pct : float
        0.0 = fully closed, 1.0 = fully open.
    """
    value = max(0, min(100, int(open_pct * 100)))
    payload = bytes([value])
    cmd = bytes([CMD_GRIPPER, len(payload)]) + payload
    return bytes([FRAME_START]) + cmd + bytes([_checksum(cmd)])


def decode_frame(data: bytes):
    """
    Attempt to decode the first complete frame in *data*.

    Returns
    -------
    tuple (cmd_id, payload) on success, or None if the frame is invalid /
    incomplete.
    """
    if len(data) < 4:
        return None
    if data[0] != FRAME_START:
        return None
    cmd_id = data[1]
    payload_len = data[2]
    if len(data) < 3 + payload_len + 1:
        return None  # not enough bytes yet
    payload = data[3: 3 + payload_len]
    checksum = data[3 + payload_len]
    body = data[1: 3 + payload_len]
    if _checksum(body) != checksum:
        return None  # checksum mismatch
    return cmd_id, payload
