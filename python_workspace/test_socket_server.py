import socket
import json
import time
import struct

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

# TODO: Add an exception if port is in use

def handle_command(cmd, *args):
    # if cmd.startswith("testy"):
    #     return ("testy success")
    pass

def sock_listener():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen()
        print(f"Server listening on {HOST}:{PORT}")

        while True:
            print("waiting for clients")
            conn, addr = s.accept()  # Wait for incoming connection

            with conn:
                print(f"Client {addr} is connected to the server.")
                while True:
                    try:
                        # This block will accumulate data
                        packet = accumulate_packet(conn)
                        print("accumulated packet")
                        if packet is None:
                            print("Client disconnected.")
                            break

                        response = packet_to_cmd(packet)

                        packet_to_send = struct.pack('<5f', 69.69, 42.69, 1, 2, 3)

                        print(f"final response: {packet_to_send}")
                        
                        if (packet_to_send):
                            conn.sendall(packet_to_send)
                    except Exception as e:
                        print(f"Socket error: {e}")
                        break
            print(f"Client {addr} disconnected.")

def accumulate_packet(connection):
    data_store = b''

    while (len(data_store) < BYTE_FRAME_LENGTH):
        # print(f"accumulating data: {len(data_store)}")
        packet = connection.recv(BYTE_FRAME_LENGTH - len(data_store))
        if not packet:
            # Client disconnected before full frame
            if not data_store:
                print("Client disconnected before sending any data.")
                return None
            else:
                print("Client disconnected mid-frame.")
                return None
        # print(f"incoming packet: {packet}")
        data_store += packet

    stripped_data = data_store.rstrip(b'\0')  # Clear null bytes

    return stripped_data

def packet_to_cmd(packet):
    try:
        json_str = packet.decode("utf-8")   # Decode bytes and serialize data in to a string
        json_data = json.loads(json_str)     # Re-serialize data into JSON object            
        # response = handle_command(json_data["command"], *json_data["args"])
        # print(f"packet_to_cmd response: {response}")
        # return response
    except:
        print("bro wtf")

def main():
    sock_listener()

if __name__=="__main__":
    main()