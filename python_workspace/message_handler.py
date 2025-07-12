import struct
import signal
import sys

from protocol_constants import ProtocolConstants, MessageTypes
from protocol_parser import ProtocolParser
from robot_class import RobotArm

class MessageHandler:
    def __init__(self):
        self.robot_arm = RobotArm()

    def handle_message(self, incoming_message: bytes) -> bytes:
        message_byte = incoming_message[0]

        if (len(incoming_message) > 1):
            payload = incoming_message[1:]
        else:
            payload = b''

        match (message_byte):
            case (MessageTypes.READ_JOINT_ANGLES.value):
                return self.handle_read_joint_angles()
            case _:
                print("No message found!")
                return None

    def handle_read_joint_angles(self):
        angles = self.robot_arm.read_joint_angle(0)
        angles_to_bytes = ProtocolParser.encode_joint_angles(angles)

        return angles_to_bytes