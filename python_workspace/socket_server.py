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
    """Socket server for TCP communication."""

    def __init__(self, config_file: str = "config.json"):
        # Initialize configuration settings
        self.config = self._load_config(config_file)
        self.HOST_ADDR = self.config["network_settings"]["HOST"]
        self.NETWORK_PORT = self.config["network_settings"]["PORT"]
        self.client_connected = False
        self.BYTE_FRAME_LENGTH = ProtocolConstants.FRAME_BUFFER_LENGTH

        # Initialize communication classes and register handlers
        self.message_handler = MessageHandler()
        self._register_handlers()

    def _load_config(self, config_file: str):
        """Load network settings from configuration file.
        
        Takes the path of the configuration file as an argument.

        Returns the deserialized JSON object.
        """

        try:
            with open(config_file, "r") as f:
                config = json.load(f)
                return config
        except:
            print(f"Configuration JSON file {config_file} not found.")
            exit()

    def _register_handlers(self):
        """ Register all message handler callables to the MessageHandler instance."""

        if self.message_handler:
            self.message_handler.register_handler(MessageTypes.CONNECT, self.message_handler.handle_connect)
            self.message_handler.register_handler(MessageTypes.DISCONNECT, self.message_handler.handle_disconnect)
            self.message_handler.register_handler(MessageTypes.HOME, self.message_handler.handle_home)
            self.message_handler.register_handler(MessageTypes.DISABLE, self.message_handler.handle_disable)
            self.message_handler.register_handler(MessageTypes.READ_JOINT_ANGLES, self.message_handler.handle_read_joint_angles)
            self.message_handler.register_handler(MessageTypes.UPDATE_EE_POSITION, self.message_handler.handle_update_EE_pos)
            self.message_handler.register_handler(MessageTypes.MOVE_X, self.message_handler.handle_move_x)
            self.message_handler.register_handler(MessageTypes.MOVE_Y, self.message_handler.handle_move_y)
            self.message_handler.register_handler(MessageTypes.MOVE_Z, self.message_handler.handle_move_z)
            self.message_handler.register_handler(MessageTypes.SAVE_CURRENT_POSITION, self.message_handler.handle_save_current_position)
            self.message_handler.register_handler(MessageTypes.MOVE_TO_POSITION, self.message_handler.handle_move_to_position)

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
        """Call the protocol handler to receive and process the incoming client messages.
        
        This method accumulates bytes until the buffer is filled, the forwards the message
        to the MessageHandler class. When a response from the MessageHandler class is received,
        this response (in the form of data) is sent back to the client.
        """

        incoming_packet = self._accumulate_packet()
        response_data = self.message_handler.handle_message(incoming_packet)

        print(f"Response: {response_data}")

        if (response_data):
            self.client.sendall(response_data)

        # Special case
        if (response_data == ProtocolParser.encode_message(MessageTypes.DISCONNECT)):
            self.client_connected = False
            print(f"Client disconnected.")

    def _accumulate_packet(self) -> bytes:
        """Receive bytes until the byte frame is filled.

        A constant "frame" size is defined in PROTOCOL_CONSTANTS. When a message is received,
        this method continuously tries to receive bytes until the length of the data_store
        variable is equal to the frame size.
        
        TODO: Handle cases where the last byte(s) could be null bytes. Right now it could
        strip too much even if trailing nulls are valid parts of the message.
        """

        data_store = b''

        while (len(data_store) < self.BYTE_FRAME_LENGTH):
            packet = self.client.recv(self.BYTE_FRAME_LENGTH - len(data_store))
            data_store += packet

        stripped_data = data_store.rstrip(b'\0')    # Clear null byte padding

        return stripped_data