"""Socket server for communicating via TCP.

For now, it will only handle one client.

This file sets up the socket server, listens for clients, then listens for incoming messages.
When messages are received, they are routed to the message handler, and the response from the message handler
is sent backt to the client.
"""

import socket
import json

from message_handler import MessageHandler
from protocol_constants import ProtocolConstants, MessageTypes
from protocol_parser import ProtocolParser

class SocketServer:
    BYTE_FRAME_LENGTH = 64  # Test with 64 for now

    def __init__(self, config_file="config.json"):
        # Initialize configuration settings
        self.config = self._load_config(config_file)
        self.HOST_ADDR = self.config["network_settings"]["HOST"]
        self.NETWORK_PORT = self.config["network_settings"]["PORT"]
        self.client_connected = False

        # Initialize communication classes and register handlers
        self.message_handler = MessageHandler()
        self._register_handlers()

    def _load_config(self, config_file):
        """Load network settings from configuration file."""
        try:
            with open(config_file, "r") as f:
                config = json.load(f)
                return config
        except:
            print(f"Configuration JSON file {config_file} not found.")
            exit()

    def _register_handlers(self):
        if self.message_handler:
            self.message_handler.register_handler(MessageTypes.CONNECT, self.message_handler.handle_connect)
            self.message_handler.register_handler(MessageTypes.DISCONNECT, self.message_handler.handle_disconnect)
            self.message_handler.register_handler(MessageTypes.HOME, self.message_handler.handle_home)
            self.message_handler.register_handler(MessageTypes.DISABLE, self.message_handler.handle_disable)
            self.message_handler.register_handler(MessageTypes.READ_JOINT_ANGLES, self.message_handler.handle_read_joint_angles)

    def start(self):
        """Start the socket server and listen for client connections."""
        self.socket_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket_server.bind((self.HOST_ADDR, self.NETWORK_PORT))
        self.socket_server.listen()

        print(f"Server started. Listening on {self.HOST_ADDR}:{self.NETWORK_PORT}")

        while True:
            print("Waiting for clients...")
            self.client, client_addr = self.socket_server.accept() # Wait for incoming connection

            self.client_connected = True
            print(f"Client connected from {client_addr}.")

            while (self.client_connected):
                self._handle_client_message()   # Listen for and handle client messages indefinitely

    def stop(self):
        """Stop the socket server."""
        self.socket_server.close()
        print("Socket server connection closed.")

    def _handle_client_message(self):
        """Call the protocol handler to receive and process the incoming client messages."""
        incoming_packet = self._accumulate_packet()
        response_data = self.message_handler.handle_message(incoming_packet)

        print(f"Response: {response_data}")

        if (response_data):
            self.client.sendall(response_data)

        if (response_data == ProtocolParser.encode_message(MessageTypes.DISCONNECT)):
            self.client_connected = False
            print(f"Client disconnected.")

    def _accumulate_packet(self) -> bytes:
        """Receive bytes until the byte frame is filled."""
        data_store = b''

        while (len(data_store) < self.BYTE_FRAME_LENGTH):
            # Try to receive the number of bytes required to fill the frame
            packet = self.client.recv(self.BYTE_FRAME_LENGTH - len(data_store))
            data_store += packet
        
        stripped_data = data_store.rstrip(b'\0')    # Clear null byte padding

        return stripped_data