"""
Web sockets server to communicate with Three.js visualizer client.

https://websockets.readthedocs.io/en/stable/
"""

import asyncio
import websockets
import traceback

from communication.websocket.websocket_message_handler import WebSocketMsgHandler

class WebSocketServer:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.clients = set()
        self.server_task = None

        self.websocket_msg_handler = WebSocketMsgHandler(self)

    async def _connection_handler(self, websocket):
        """
        This method is called automatically when a client is connected. The argument,
        websocket, is automatically passed internally by the websockets library.
        """
        print(f"connection_handler triggered")
        self.clients.add(websocket)
        print(f"Client connected to websocket server. Client address: {websocket}")

        try:
            async for message in websocket:
                await self.handle_message(message)
        finally:
            await self.clients.remove(websocket)

    async def handle_message(self, message):
        await self.websocket_msg_handler.handle_message(message)

    def start_server(self):
        """Start the server (non-async method)"""
        if self.server_task is None:
            self.server_task = asyncio.create_task(self._start_server_async())

    async def _start_server_async(self):
        """
        Coroutine to start the server.
        
        serve() takes 3 arguments.
            1. The connection handler function (what happens when a client connects)
            2. The host
            3. The port to listen on

        Invoking serve() within a context manager ensures that the server is destroyed when the program terminates.
        """

        async with websockets.serve(self._connection_handler, self.host, self.port) as server:
            print(f"Started websocket server on {self.host}:{self.port}")
            await server.wait_closed()

    async def broadcast(self, message):
        """
        Send messages to clients
        """
        print(f"clients: {self.clients}")
        if self.clients:
            print("Broadcasting message to clients.")
            await asyncio.gather(*[client.send(message) for client in self.clients], return_exceptions=True)    # ???
            print("Broadcasted message to clients.")