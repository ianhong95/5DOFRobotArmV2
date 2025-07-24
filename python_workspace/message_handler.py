"""Message routing class.

This class is responsible for calling encode/decode on messages, then routing the message to the appropriate handler method.
"""
from typing import Callable, Dict, Any  # Only for type hinting; built-in versions would create objects instead
from math import degrees
import struct

import numpy as np

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

        response = self.message_handlers[message_type_value](payload)
        
        return response
    
    def handle_connect(self, payload: bytes) -> bytes:
        return (ProtocolParser.encode_message(MessageTypes.CONNECT))

    def handle_disconnect(self, payload: bytes) -> bytes:
        return (ProtocolParser.encode_message(MessageTypes.DISCONNECT))

    def handle_home(self, payload: bytes) -> bytes:
        self.robot_arm.home()
        angles_in_radians = self.robot_arm.read_joint_angle(0)
        angles_in_degrees = [round(degrees(angle), 2) for angle in angles_in_radians]
        xyz_position = (self.robot_arm.get_ee_pos()).tolist()
        rounded_xyz_position = [round(coordinate, 2) for coordinate in xyz_position]
        
        return (ProtocolParser.encode_message(MessageTypes.HOME, angles_in_degrees + rounded_xyz_position))

    def handle_disable(self, payload: bytes) -> bytes:
        self.robot_arm.disable_servo(0)

        return (ProtocolParser.encode_message(MessageTypes.DISABLE))

    def handle_read_joint_angles(self, payload: bytes) -> bytes:
        """Read the robot's joint angles and convert them to degrees."""
        angles_in_radians = self.robot_arm.read_joint_angle(0)
        angles_in_degrees = [round(degrees(angle), 2) for angle in angles_in_radians]
        encoded_message = ProtocolParser.encode_message(MessageTypes.READ_JOINT_ANGLES, angles_in_degrees)

        return encoded_message
    
    def handle_update_EE_pos(self, payload: bytes) -> bytes:
        """Read the end effector's position in Cartesian coordinates (x,y,z)."""

        xyz_position = (self.robot_arm.get_ee_pos()).tolist()
        rounded_xyz_position = [round(coordinate, 2) for coordinate in xyz_position]

        return (ProtocolParser.encode_message(MessageTypes.UPDATE_EE_POSITION, rounded_xyz_position))
    
    def handle_move_x(self, payload: bytes) -> bytes:
        """Move in the x direction by a specified distance from the current position."""
        
        x_distance = (struct.unpack('f', payload))[0]

        self.robot_arm.move_x(x_distance)

    def handle_move_y(self, payload: bytes) -> bytes:
        """Move in the y direction by a specified distance from the current position."""
        
        y_distance = (struct.unpack('f', payload))[0]

        self.robot_arm.move_y(y_distance)

    def handle_move_z(self, payload: bytes) -> bytes:
        """Move in the z direction by a specified distance from the current position."""
        
        z_distance = (struct.unpack('f', payload))[0]

        self.robot_arm.move_z(z_distance)