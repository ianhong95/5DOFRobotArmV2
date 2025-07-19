"""Message routing class.

This class is responsible for calling encode/decode on messages, then routing the message to the appropriate handler method.
"""
from typing import Callable, Dict, Any  # Only for type hinting; built-in versions would create objects instead
from math import degrees

from protocol_constants import ProtocolConstants, MessageTypes
from protocol_parser import ProtocolParser
from robot_class import RobotArm

class MessageHandler:
    def __init__(self):
        # Dictionary to map handler methods to message types so we can have just one generic "handle_message" method
        self.message_handlers: Dict[bytes, Callable] = {}

        # Initialize robot arm instance
        self.robot_arm = RobotArm()

    def register_handler(self, message_type: bytes, handler: Callable):
        """Register an entry to the message_handlers dictionary."""
        self.message_handlers[message_type] = handler

    def handle_message(self, incoming_message: bytes) -> bytes:
        """Extract the message type and payload, then call the appropriate handler method."""
        message_type_value, payload = ProtocolParser.decode_message(incoming_message)
        print(f'message_type_key: {message_type_value}')

        response = self.message_handlers[message_type_value](payload)
        
        return response
    
    def handle_connect(self, payload: bytes) -> bytes:
        return(ProtocolParser.encode_message(MessageTypes.CONNECT, MessageTypes.OK))

    def handle_disconnect(self, payload: bytes) -> bytes:
        return(ProtocolParser.encode_message(MessageTypes.DISCONNECT, MessageTypes.OK))

    def handle_home(self, payload: bytes) -> bytes:
        self.robot_arm.home()
        return(ProtocolParser.encode_message(MessageTypes.HOME, MessageTypes.OK))

    def handle_disable(self, payload: bytes) -> bytes:
        self.robot_arm.disable_servo(0)
        return(ProtocolParser.encode_message(MessageTypes.DISABLE, MessageTypes.OK))

    def handle_read_joint_angles(self, payload: bytes) -> bytes:
        """Read the robot's joint angles and convert them to degrees."""
        angles_in_radians = self.robot_arm.read_joint_angle(0)
        angles_in_degrees = [degrees(angle) for angle in angles_in_radians]
        byte_angles = ProtocolParser.encode_joint_angles(angles_in_degrees)

        return(ProtocolParser.encode_message(MessageTypes.READ_JOINT_ANGLES, byte_angles))