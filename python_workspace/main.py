"""
Main unifying file to run all services.

TODO:
    - Clean up config loading in all files.
    - Remove all hardcoded hosts and ports
    - Figure out cleaner solution to argument drilling
    - Race conditions galore
"""

import asyncio

from tcp_socket_server import TCPSocketServer
from communication.websocket.websocket_server import WebSocketServer

async def main():
    websocket_server = WebSocketServer("0.0.0.0", 60003)
    tcp_socket_server = TCPSocketServer(websocket_server)

    start_websocket_server = asyncio.create_task(websocket_server._start_server_async())
    start_tcp_server = asyncio.create_task(tcp_socket_server.start())

    await asyncio.gather(start_websocket_server, start_tcp_server)

if __name__=="__main__":
    asyncio.run(main())