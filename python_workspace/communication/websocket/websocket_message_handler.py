"""
Routing class to forward incoming TCP messages to corresponding websocket message handler methods.
"""

import json
import asyncio

from communication.websocket.websocket_protocol_constants import WebSocketMsgTypes, JSONKeys

class WebSocketMsgHandler:
    def __init__(self, websocket_server):
        self.websocket_server = websocket_server

        self.handlers = {
            WebSocketMsgTypes.HOME: self._handle_home,
            WebSocketMsgTypes.READ_JOINT_ANGLES: self._handle_read_joint_angles
            # etc
        }

    async def handle_message(self, message):
        """
        Pass as an argument the full message including the type and payload.
        """

        message_type = message[JSONKeys.TYPE]
        await self.handlers[message_type](message[JSONKeys.PAYLOAD])

    async def _handle_read_joint_angles(self, payload):
        """Handler for reading joint angles."""

        message = {
            JSONKeys.TYPE: WebSocketMsgTypes.READ_JOINT_ANGLES,
            JSONKeys.PAYLOAD: payload
        }

        await self.websocket_server.broadcast(json.dumps(message))
        

    async def _handle_home(self, payload):
        """Handler for homing the robot."""

        message = {
            JSONKeys.TYPE: WebSocketMsgTypes.HOME,
            JSONKeys.PAYLOAD: payload
        }

        print("Broadcasting HOME message")
        await self.websocket_server.broadcast(json.dumps(message))
        print("Broadcasted HOME message")

