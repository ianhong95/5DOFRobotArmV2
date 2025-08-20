"""Socket server for communicating via TCP.

For now, it will only handle one client.

This file sets up the socket server, listens for clients, then listens for incoming messages.
When messages are received, they are routed to the message handler, and the response from the message handler
is sent backt to the client.
"""

# import socket
import json
import asyncio

from message_handler import MessageHandler
from protocol_constants import ProtocolConstants, MessageTypes
from protocol_parser import ProtocolParser

class TCPSocketServer:
    """
    Socket server for TCP communication.
    
    TODO: Move all the config loading to main.py
    """

    def __init__(self, config_file: str):
        # Initialize configuration settings
        self.config = self._load_config(config_file)
        self.HOST_ADDR = self.config["network_settings"]["HOST"]
        self.NETWORK_PORT = self.config["network_settings"]["PORT"]
        self.client_connected = False
        self.BYTE_FRAME_LENGTH = ProtocolConstants.FRAME_BUFFER_LENGTH
        self._ENV = self.config["env"]["SIMULATION"]
        # self.on_message_callback = on_message_callback

        # Initialize communication classes and register handlers
        self.message_handler = MessageHandler(self._ENV)
        self._register_handlers()

        # Initialize empty set (unordered, unique) to store client connections
        self.clients = set()

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
            self.message_handler.register_handler(MessageTypes.PLAY_CURRENT_SEQUENCE, self.message_handler.handle_play_current_sequence)
            self.message_handler.register_handler(MessageTypes.OPEN_GRIPPER, self.message_handler.handle_open_gripper)
            self.message_handler.register_handler(MessageTypes.CLOSE_GRIPPER, self.message_handler.handle_close_gripper)

        print("Message handlers registered.")

    async def start(self):
        """
        Start the socket server and listen for client connections.
        """

        self.server = await asyncio.start_server(
            self._handle_client_connection,
            self.HOST_ADDR,
            self.NETWORK_PORT
        )
        print(f"Server started. Listening on {self.HOST_ADDR}:{self.NETWORK_PORT}")

        await self.server.serve_forever()

    async def _handle_client_connection(self, client_reader: asyncio.StreamReader, client_writer: asyncio.StreamWriter):
        """
        This method is called when a client connects to the server.
        """
        addr = client_writer.get_extra_info('peername')
        print(f"Client {addr} connected!")

        try: 
            while True:
                await self._handle_client_message(client_reader, client_writer)
        except (ConnectionResetError, asyncio.IncompleteReadError):
            print(f"Client {addr} disconnected.")
        except Exception as e:
            print(f"Error handling client {addr}: {e}")
        finally:
            client_writer.close()
            await client_writer.wait_closed()
        
    async def stop(self):
        """Stop the socket server."""

        self.server.close()
        print("Socket server connection closed.")

    async def _handle_client_message(self, client_reader: asyncio.StreamReader, client_writer: asyncio.StreamWriter):
        """
        Call the protocol handler to receive and process the incoming client messages.
        
        This method accumulates bytes until the buffer is filled, the forwards the message
        to the MessageHandler class. When a response from the MessageHandler class is received,
        this response (in the form of data) is sent back to the client.
        """

        incoming_packet = await self._accumulate_packet(client_reader)
        preprocessed_message = self._strip_data(incoming_packet)

        response_data = self.message_handler.handle_message(preprocessed_message)

        print(f"Response: {response_data}")

        if (response_data):
            client_writer.write(response_data)
            await client_writer.drain()

        # Special case
        if (response_data == ProtocolParser.encode_message(MessageTypes.DISCONNECT)):
            self.client_connected = False
            print(f"Client disconnect message received.")

        # TODO: Placeholder for now. Implement this after writing websocket server.
        # await self.on_message_callback()

    async def _accumulate_packet(self, client_reader) -> bytes:
        """Receive bytes until the byte frame is filled.

        A constant "frame" size is defined in PROTOCOL_CONSTANTS. When a message is received,
        this method continuously tries to receive bytes until the length of the data_store
        variable is equal to the frame size.
        """

        data_store = b''

        while (len(data_store) < self.BYTE_FRAME_LENGTH):
            packet = await client_reader.read(self.BYTE_FRAME_LENGTH - len(data_store))

            if not packet:
                raise ConnectionResetError("Client disconnected during message transmission.")
            
            data_store += packet

        return data_store
    
    def _strip_data(self, data: bytes) -> bytes:
        """Strips message padding to keep the message length according to its type."""

        message_type = ProtocolParser.get_message_type(data)
        message_length = ProtocolParser.get_message_length(message_type)
        stripped_message = ProtocolParser.strip_padding(data, message_length)

        return stripped_message