"""Parser for protocol messages.

This class takes in data from the message handler, processes it, and returns the appropriate
pieces of data based on the message type.
"""

import struct
from protocol_constants import ProtocolConstants, MessageTypes, ResponseTypes

class ProtocolParser:
    @staticmethod
    def encode_message(message_type: MessageTypes, payload: list[float] = []) -> bytes:
        """Encode a message given a message type and payload.
        
        The message is padded to 64 bytes regardless of the payload size.
        TODO: Test for successful encoding and actually use the response.
        """
        print(f"payload: {payload}")
        packed_payload = struct.pack(f'<{len(payload)}f', *payload)

        data: bytes = message_type + ResponseTypes.OK + packed_payload

        if len(data) < ProtocolConstants.FRAME_BUFFER_LENGTH:
            padded_message = data.ljust(64, b'\x00')

        return padded_message
    
    @staticmethod
    def encode_joint_angles(payload: list[float]) -> bytes:
        """Encode joint angles (degrees) to bytes, and prepends the READ_JOINT_ANGLES message type."""
        message_type = MessageTypes.READ_JOINT_ANGLES
        packed_angles = struct.pack('<5f', *payload)

        return message_type + packed_angles
    
    @staticmethod
    def decode_message(incoming_message: bytes):
        """Decode a message (bytes) to extract the message type and payload into a format that can be used by other classes."""
        message_byte = incoming_message

        if (len(incoming_message) > 1):
            payload = incoming_message[1:]
        else:
            payload = b''

        print(f"message_byte: {message_byte}")
        print(f"payload: {payload}")

        return message_byte, payload  