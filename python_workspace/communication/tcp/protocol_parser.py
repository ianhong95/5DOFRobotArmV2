"""Parser for protocol messages.

This class takes in data from the message handler, processes it, and returns the appropriate
pieces of data based on the message type.
"""

import struct
from communication.tcp.protocol_constants import ProtocolConstants, MessageTypes, ResponseTypes

class ProtocolParser:
    """A class to encode/decode messages according to the protocol defined in ProtocolConstants."""

    @staticmethod
    def encode_message(message_type: MessageTypes, payload: list[float | int] = []) -> bytes:
        """Encode a message given a message type and payload.
        
        The message is padded to 64 bytes regardless of the payload size.
        TODO: Test for successful encoding and actually use the response.

        Returns the padded message.
        """
        
        packed_payload = struct.pack(f'<{len(payload)}f', *payload)

        data: bytes = message_type + ResponseTypes.OK + packed_payload

        if len(data) < ProtocolConstants.FRAME_BUFFER_LENGTH:
            padded_message = data.ljust(ProtocolConstants.FRAME_BUFFER_LENGTH, b'\x00')

        return padded_message
    
    @staticmethod
    def encode_save_message(message_type: MessageTypes, payload: list[float | int] = []) -> bytes:
        """Encode a message given a message type and payload.
        
        The message is padded to 64 bytes regardless of the payload size.
        TODO: Test for successful encoding and actually use the response.

        Returns the padded message.
        """
        
        packed_payload = struct.pack(f'<i3f', *payload)

        data: bytes = message_type + ResponseTypes.OK + packed_payload

        if len(data) < ProtocolConstants.FRAME_BUFFER_LENGTH:
            padded_message = data.ljust(ProtocolConstants.FRAME_BUFFER_LENGTH, b'\x00')

        return padded_message
    
    @staticmethod
    def encode_joint_angles(payload: list[float]) -> bytes:
        """Encode joint angles (degrees) to bytes, and prepends the READ_JOINT_ANGLES message type.
        
        Returns a single byte array with the message type first, followed by the bytes representation of the angles.
        """
        message_type = MessageTypes.READ_JOINT_ANGLES
        packed_angles = struct.pack('<5f', *payload)

        return message_type + packed_angles
    
    @staticmethod
    def decode_message(incoming_message: bytes) -> bytes:
        """Decode a message (bytes) to extract the message type and payload into a format that can be used by other classes.
        
        At this point, the message has been stripped of all trailing null bytes.

        Returns the message byte and payload.
        """
        # message_byte = incoming_message[0]    DON'T DO THIS, it returns the INTEGER representation
        message_byte = incoming_message[:1]     # Slicing returns the bytes representation, which is what we want.

        if (len(incoming_message) > 1):
            payload = incoming_message[1:]
        else:
            payload = b''

        print(f"message_byte: {message_byte}")
        print(f"payload: {payload}")

        return message_byte, payload
    
    @staticmethod
    def get_message_type(incoming_message: bytes) -> bytes:
        """Returns the message type (first byte) of an incoming message."""

        return incoming_message[:1]
    
    @staticmethod
    def get_message_length(message_type: bytes) -> int:
        """Returns the expected length of the message based on its type."""

        return MessageTypes.LENGTH[message_type]
    
    @staticmethod
    def strip_padding(incoming_message: bytes, expected_length: int) -> bytes:
        """Keep expected message length and remove padding (trailing null bytes)."""

        return incoming_message[:expected_length]
