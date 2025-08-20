import asyncio

from message_handler import MessageHandler
from protocol_parser import ProtocolParser
from protocol_constants import ProtocolConstants, MessageTypes
from robot_class import RobotArm
from socket_server import TCPSocketServer


async def main():
    tcp_socket_server = TCPSocketServer("config.json")
    await tcp_socket_server.start()

def on_message_callback():
    print("Client connected")

if __name__=="__main__":
    asyncio.run(main())