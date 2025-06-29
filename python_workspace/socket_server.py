import socket
import json
from robot_class import RobotArm


CONFIG_FILE = "config.json"
try:
    with open(CONFIG_FILE, "r") as f:
        config = json.load(f)
except:
    print(f"{CONFIG_FILE} not found.")
    exit()

HOST = config["network_settings"]["HOST"]
PORT = config["network_settings"]["PORT"]
BYTE_FRAME_LENGTH = 64

ROBOT = RobotArm()

CMD_MAP = {
    "connect": "initialize",
    "home": ROBOT.home,
    "disable": ROBOT.disable_servo,
    "move_x": ROBOT.move_x,
    "move_y": ROBOT.move_y,
    "move_z": ROBOT.move_z
}

# TODO: Add an exception if port is in use

def handle_command(cmd, *args):
    # if cmd.startswith("testy"):
    #     return ("testy success")
    func = CMD_MAP.get(cmd)
    print(*args)
    if func:
        func(*args)
        print(f"Executed {func}.")


def sock_listener():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen()
        print(f"Server listening on {HOST}:{PORT}")

        conn, addr = s.accept()  # Wait for incoming connection
        with conn:
            print(f"Client {addr} is connected to the server.")
            while True:
                # This block will accumulate data
                packet = accumulate_packet(conn)
                response = packet_to_cmd(packet)

        conn.close()

def accumulate_packet(connection):
    data_store = b''

    while (len(data_store) < BYTE_FRAME_LENGTH):
        print(f"accumulating data: {len(data_store)}")
        packet = connection.recv(BYTE_FRAME_LENGTH - len(data_store))
        print(f"incoming packet: {packet}")
        data_store += packet

    stripped_data = data_store.rstrip(b'\0')  # Clear null bytes

    print(f"64 bytes accumulated!")
    return stripped_data

def packet_to_cmd(packet):
    try:
        json_str = packet.decode("utf-8")   # Decode bytes and serialize data in to a string
        json_data = json.loads(json_str)     # Re-serialize data into JSON object            
        response = handle_command(json_data["command"], *json_data["args"])
        return response
    except:
        print("bro wtf")

def main():
    sock_listener()

if __name__=="__main__":
    main()