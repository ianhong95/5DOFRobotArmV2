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
    MOVE_J1 = b'\x1E'
    MOVE_J2 = b'\x1F'
    MOVE_J3 = b'\x20'
    MOVE_J4 = b'\x21'
    MOVE_J5 = b'\x22'

class ResponseTypes:
    OK = b'\x64'
    ERROR = b'\x65'

# Message structures
class ProtocolConstants:
    FRAME_BUFFER_LENGTH = 64
    JOINT_ANGLES_RESP_SIZE = 20
