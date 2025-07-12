from enum import IntEnum

# Message type constants
class MessageTypes(IntEnum):
    CONNECT = 0x01,
    DISCONNECT = 0x02,
    HOME = 0x0A,
    DISABLE = 0x0B,
    READ_JOINT_ANGLES = 0x0C,
    UPDATE_EE_POSITION = 0x0D,
    UPDATE_EE_ORIENTATION = 0x0E,
    MOVE_X = 0x14,
    MOVE_Y = 0x15,
    MOVE_Z = 0x16,
    MOVE_J1 = 0x1E,
    MOVE_J2 = 0x1F,
    MOVE_J3 = 0x20,
    MOVE_J4 = 0x21,
    MOVE_J5 = 0x22


# Message structures
class ProtocolConstants:
    JOINT_ANGLES_RESP_SIZE = 20