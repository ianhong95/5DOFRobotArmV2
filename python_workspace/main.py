from message_handler import MessageHandler
from protocol_parser import ProtocolParser
from protocol_constants import ProtocolConstants, MessageTypes
from robot_class import RobotArm
from socket_server import SocketServer

def main():
    socket_server = SocketServer()
    socket_server.start()

if __name__=="__main__":
    main()