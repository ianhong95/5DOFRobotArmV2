"""
Websocket protocol constants for communicatio between TCP server and websocket server.
"""

class WebSocketMsgTypes:
    READ_JOINT_ANGLES = "read_joint_angles"
    HOME = "home"
    MOVE_X = "move_x"
    MOVE_Y = "move_y"
    MOVE_Z = "move_z"

class JSONKeys:
    TYPE = "type"
    PAYLOAD = "payload"
