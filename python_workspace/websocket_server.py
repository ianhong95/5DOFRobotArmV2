"""
Web sockets server to communicate with Three.js visualizer client.

https://websockets.readthedocs.io/en/stable/
"""

import asyncio
import websockets

class WebSocketServer:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.clients = set()

    async def echo(self, websocket):
        async for message in websocket:
            await websocket.send(message)

    async def connection_handler(self, websocket):
        """
        This method should run an infinite loop as long as the client is connected.
        
        """
        while (True):
            message = await websocket.recv()
            print(message)

    async def main(self):
        """
        Coroutine to start the server.
        
        serve() takes 3 arguments.
            1. The connection handler function (what happens when a client connects)
            2. The host
            3. The port to listen on

        Invoking serve() within a context manager ensures that the server is destroyed when the program terminates.
        """
        async with websockets.serve(self.echo, "localhost", 60002) as server:
            await server.serve_forever()

    async def broadcast(self, message):
        """
        Send messages to clients
        """

        if self.clients:
            await asyncio.gather(*[client.send(message) for client in self.clients])    # ???