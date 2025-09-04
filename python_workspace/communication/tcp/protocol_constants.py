# Message type constants.
class MessageTypes:
    CONNECT = b'\x01'
    DISCONNECT = b'\x02'
    HOME = b'\x0A'
    DISABLE = b'\x0B'
    READ_JOINT_ANGLES = b'\x0C'
    UPDATE_EE_POSITION = b'\x0D'
    UPDATE_EE_ORIENTATION = b'\x0E'
    MOVE_X = b'\x14'
    MOVE_Y = b'\x15'
    MOVE_Z = b'\x16'
    SET_J1 = b'\x1E'
    SET_J2 = b'\x1F'
    SET_J3 = b'\x20'
    SET_J4 = b'\x21'
    SET_J5 = b'\x22'
    OPEN_GRIPPER = b'\x23'
    CLOSE_GRIPPER = b'\x24'
    READ_GRIPPER_STATE = b'\x25'
    SAVE_CURRENT_POSITION = b'\x28'
    SAVE_SEQUENCE = b'\x29'
    MOVE_TO_POSITION = b'\x2A'
    PLAY_CURRENT_SEQUENCE = b'\x2B'

    # Length of INCOMING messages
    LENGTH = {
        CONNECT: 1,
        DISCONNECT: 1,
        HOME: 1,
        DISABLE: 1,
        READ_JOINT_ANGLES: 1,
        UPDATE_EE_POSITION: 1,
        UPDATE_EE_ORIENTATION: 1,
        MOVE_X: 5,
        MOVE_Y: 5,
        MOVE_Z: 5,
        SET_J1: 5,
        SET_J2: 5,
        SET_J3: 5,
        SET_J4: 5,
        SET_J5: 5,
        OPEN_GRIPPER: 1,
        CLOSE_GRIPPER: 1,
        SAVE_CURRENT_POSITION: 13,
        SAVE_SEQUENCE: 53,   # Message type (1), number of moves (4), up to 12 moves in sequence (48)
        MOVE_TO_POSITION: 5,
        PLAY_CURRENT_SEQUENCE: 61   # Message type (1), number of moves (4), up to 14 moves in sequence (56)
    }

class ResponseTypes:
    OK = b'\x64'
    ERROR = b'\x65'

# Message structures
class ProtocolConstants:
    FRAME_BUFFER_LENGTH = 64
    JOINT_ANGLES_RESP_SIZE = 20
