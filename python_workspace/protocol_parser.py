import struct
from protocol_constants import ProtocolConstants, MessageTypes

class ProtocolParser:
    @staticmethod
    def encode_message(message_type: MessageTypes, payload=None) -> bytes:
        """Encode a message given a message type and payload."""
        return message_type.value + payload
    
    @staticmethod
    def decode_message():
        return
    
    @staticmethod
    def encode_joint_angles(payload: list[float]) -> bytes:
        """Encode joint angles (degrees) to bytes, and prepends the READ_JOINT_ANGLES message type."""
        message_type = MessageTypes.READ_JOINT_ANGLES.value
        packed_angles = struct.pack("<5f", *payload)

        return message_type + packed_angles