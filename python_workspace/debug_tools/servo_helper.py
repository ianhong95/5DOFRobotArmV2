''' Python script to configure Feetech ST3215 servos.

'''

import json
import os
import sys

sys.path.append("..")
from scservo_sdk import *   # Uses SC Servo SDK library

# Get the directory of the current script (scripts/)
script_dir = os.path.dirname(os.path.abspath(__file__))

# Get the parent directory (robot_arm/)
parent_dir = os.path.abspath(os.path.join(script_dir, ".."))

# Build the path to config.json in robot_arm/
config_file = os.path.join(parent_dir, "config.json")

try:
    with open(config_file, "r") as f:
        config = json.load(f)
except:
    print(f"{config_file} not found.")
    exit()

BAUDRATE = config["serial_settings"]["baudrate"]
DEVICE = config["serial_settings"]["device"]

portHandler = PortHandler(DEVICE)
time.sleep(1)

if portHandler.openPort():
    print(f"Port opened successfully: {DEVICE}")

else:
    print("Failed to open the port")
    exit()

if portHandler.setBaudRate(BAUDRATE):
    print(f"Baudrate set to {BAUDRATE} successfully")
else:
    print("Failed to change the baudrate")
    exit()

p = sms_sts(portHandler)

activeServos = []

for id in range(50):
    sts_model_number, sts_comm_result, sts_error = p.ping(id)
    if sts_comm_result == 0:
        activeServos.append(id)                         # Add servo ID to temporary list
        br = p.readBaudrate(id)
        # p.SetID(id, 1)
        # p.writeBaudrate(id)
        print(f"Found active servo. ID: {id}. Baudrate is {br}")
