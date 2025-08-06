"""Message routing class.

This class is responsible for calling encode/decode on messages, then routing the message to the appropriate handler method.
When this class is initialized, a handler dictionary is created. This dictionary maps each handler method to a message type
so the handle_message method can select a method based on the received message type, which is a superior alternative to a
very larget match/case or conditional statement.
"""

from typing import Callable, Dict, Any  # Only for type hinting; built-in versions would create objects instead
from math import degrees
import struct
from time import sleep

import numpy as np

from protocol_constants import ProtocolConstants, MessageTypes
from protocol_parser import ProtocolParser
from robot_class import RobotArm
from saved_positions_handler import SavedPositionsDB

class MessageHandler:
    """A class for handling incoming messages and routing them to the appropriate handler method based on the message type."""

    def __init__(self):
        # Dictionary to map handler methods to message types so we can have just one generic "handle_message" method
        self.message_handlers: Dict[bytes, Callable] = {}

        # Initialize robot arm instance
        self.robot_arm = RobotArm()
        self.db = SavedPositionsDB()

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
        """Moves the robot to the home position.
        
        Returns the HOME message type, joint angles, and coordinates in Cartesian space.
        """

        self.robot_arm.home()
        angles_in_radians = self.robot_arm.read_joint_angle(0)
        angles_in_degrees = [round(degrees(angle), 2) for angle in angles_in_radians]
        xyz_position = (self.robot_arm.get_ee_pos()).tolist()
        rounded_xyz_position = [round(coordinate, 2) for coordinate in xyz_position]
        
        return (ProtocolParser.encode_message(MessageTypes.HOME, angles_in_degrees + rounded_xyz_position))

    def handle_disable(self, payload: bytes) -> bytes:
        """Disables all motors."""

        self.robot_arm.disable_servo(0)

        return (ProtocolParser.encode_message(MessageTypes.DISABLE))

    def handle_read_joint_angles(self, payload: bytes) -> bytes:
        """Read the robot's joint angles and convert them to degrees.
        
        Returns the encoded message type and joint angles (byte representation of floats).
        """

        angles_in_radians = self.robot_arm.read_joint_angle(0)
        angles_in_degrees = [round(degrees(angle), 2) for angle in angles_in_radians]
        encoded_message = ProtocolParser.encode_message(MessageTypes.READ_JOINT_ANGLES, angles_in_degrees)

        return encoded_message
    
    def handle_update_EE_pos(self, payload: bytes) -> bytes:
        """Read the end effector's position in Cartesian coordinates (x,y,z).
        
        Returns the encoded message type and Cartesian coordinates.
        """

        xyz_position = (self.robot_arm.get_ee_pos()).tolist()
        rounded_xyz_position = [round(coordinate, 2) for coordinate in xyz_position]

        return ProtocolParser.encode_message(MessageTypes.UPDATE_EE_POSITION, rounded_xyz_position)
    
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

    def handle_save_current_position(self, payload: bytes) -> bytes:
        """Save the current joint positions."""

        joint_angles_radians = self.robot_arm.read_joint_angle(0)
        joint_angles_degrees = [round(degrees(joint_angle), 2) for joint_angle in joint_angles_radians]

        xyz_position = (self.robot_arm.get_ee_pos()).tolist()
        rounded_xyz_position = [round(coord, 2) for coord in xyz_position]

        print(rounded_xyz_position + joint_angles_degrees)

        new_row_id = self.db.add_row(rounded_xyz_position + joint_angles_radians)

        output = [new_row_id] + rounded_xyz_position

        print(f"Saved position: {output}")

        return ProtocolParser.encode_save_message(MessageTypes.SAVE_CURRENT_POSITION, output)
    
    def handle_move_to_position(self, payload: bytes) -> bytes:
        """Move to the position at a given index."""
        position_entry_index = struct.unpack('<i', payload)[0]

        joint_angles = self.db.get_joint_angles_from_idx(position_entry_index)
        self.robot_arm.sync_write_angles(joint_angles)

    def handle_play_current_sequence(self, payload: bytes) -> bytes:
        """Move to the positions in the GUI table."""
        print(f"test payload: {payload}")

        unpacked_payload = struct.unpack('<15i', payload)
        id_count = unpacked_payload[0]   # Defined by message protocol
        position_ids = unpacked_payload[1:id_count + 1]  # Add 1 because the first element is the id count

        for id in position_ids:
            joint_angles = self.db.get_joint_angles_from_idx(id)
            self.robot_arm.sync_write_angles(joint_angles)
            sleep(2)