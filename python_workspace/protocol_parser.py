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
    
    def encode_joint_angles(payload: list[float]):
        return