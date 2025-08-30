"""Message routing class.

This class is responsible for calling encode/decode on messages, then routing the message to the appropriate handler method.
When this class is initialized, a handler dictionary is created. This dictionary maps each handler method to a message type
so the handle_message method can select a method based on the received message type, which is a superior alternative to a
very larget match/case or conditional statement.

TODO: Decide if this class should handle routing to both TCP and websocket servers or just one.
"""

from typing import Callable, Dict, Any  # Only for type hinting; built-in versions would create objects instead
from math import degrees
import struct
from time import sleep
import asyncio

import numpy as np

from communication.tcp.protocol_constants import ProtocolConstants, MessageTypes
from communication.tcp.protocol_parser import ProtocolParser
from robot_arm.robot_class import RobotArm
from robot_arm.saved_positions_handler import SavedPositionsDB
from robot_arm.robot_manager import RobotManager
from communication.websocket.websocket_server import WebSocketServer
from communication.websocket.websocket_protocol_constants import WebSocketMsgTypes

class TCPMessageHandler:
    """A class for handling incoming messages and routing them to the appropriate handler method based on the message type."""

    def __init__(self, websocket_server):
        self.websocket_server = websocket_server

        # Dictionary to map handler methods to message types so we can have just one generic "handle_message" method
        self.message_handlers: Dict[bytes, Callable] = {}

        # Initialize robot arm and database interface instance
        self.robot_arm = RobotManager(simulation_robot_connected=1)
        self.db = SavedPositionsDB()

    def register_handler(self, message_type: bytes, handler: Callable):
        """Register an entry to the message_handlers dictionary."""
        self.message_handlers[message_type] = handler

    async def handle_message(self, incoming_message: bytes) -> bytes:
        """Extract the message type and payload, then call the appropriate handler method."""
        message_type_value, payload = ProtocolParser.decode_message(incoming_message)

        response = await self.message_handlers[message_type_value](payload)
        
        return response
    
    def handle_connect(self, payload: bytes) -> bytes:
        print("Handling CONNECT.")
        return (ProtocolParser.encode_message(MessageTypes.CONNECT))

    def handle_disconnect(self, payload: bytes) -> bytes:
        return (ProtocolParser.encode_message(MessageTypes.DISCONNECT))

    async def handle_home(self, payload: bytes) -> bytes:
        """Moves the robot to the home position.
        
        Returns the HOME message type, joint angles, and coordinates in Cartesian space.
        """
        
        print("Handling HOME.")

        xyz_position, angles_in_degrees = self.robot_arm.home()

        await self.websocket_server.handle_message({
            "type": WebSocketMsgTypes.HOME,
            "payload": {
                "angles": angles_in_degrees,
                "position": xyz_position
            }
        })

        return (ProtocolParser.encode_message(MessageTypes.HOME, angles_in_degrees + xyz_position))

    def handle_disable(self, payload: bytes) -> bytes:
        """Disables all motors."""

        self.robot_arm.disable()

        return (ProtocolParser.encode_message(MessageTypes.DISABLE))

    def handle_read_joint_angles(self, payload: bytes) -> bytes:
        """Read the robot's joint angles and convert them to degrees.
        
        Returns the encoded message type and joint angles (byte representation of floats).
        """

        angles_in_degrees = self.robot_arm.read_joint_angles()

        encoded_message = ProtocolParser.encode_message(MessageTypes.READ_JOINT_ANGLES, angles_in_degrees)

        return encoded_message
    
    def handle_update_EE_pos(self, payload: bytes) -> bytes:
        """Read the end effector's position in Cartesian coordinates (x,y,z).
        
        Returns the encoded message type and Cartesian coordinates.
        """

        rounded_xyz_position = self.robot_arm.get_ee_pos()

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

        xyz_position, joint_angles_degrees = self.robot_arm.save_current_position()

        new_row_id = self.db.add_row(xyz_position + joint_angles_degrees)

        output = [new_row_id] + xyz_position

        print(f"Saved position: {output}")

        return ProtocolParser.encode_save_message(MessageTypes.SAVE_CURRENT_POSITION, output)
    
    def handle_move_to_position(self, payload: bytes) -> bytes:
        """Move to the position at a given index."""
        position_entry_index = struct.unpack('<i', payload)[0]

        joint_angles = self.db.get_joint_angles_from_idx(position_entry_index)
        self.robot_arm.write_joint_angles(joint_angles)

    def handle_play_current_sequence(self, payload: bytes) -> bytes:
        """Move to the positions in the GUI table."""

        unpacked_payload = struct.unpack('<15i', payload)
        id_count = unpacked_payload[0]   # Defined by message protocol
        position_ids = unpacked_payload[1:id_count + 1]  # Add 1 because the first element is the id count

        for id in position_ids:
            joint_angles = self.db.get_joint_angles_from_idx(id)
            self.robot_arm.write_joint_angles(joint_angles)
            sleep(2)

    def handle_open_gripper(self, payload: bytes) -> bytes:
        """Sets the gripper to the fully open position."""

        self.robot_arm.set_gripper(True)

    def handle_close_gripper(self, payload: bytes) -> bytes:
        """Sets the gripper to the fully closed position"""

        self.robot_arm.set_gripper(False)