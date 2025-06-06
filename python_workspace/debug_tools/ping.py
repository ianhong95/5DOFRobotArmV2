import sys
import argparse
import json

from scservo_sdk import *   # Uses SC Servo SDK library


def parseArguments():
    # Create parser object
    parser = argparse.ArgumentParser(
        prog="python3 ping.py",
        usage=f"%(prog)s [options]",
        description="CLI tool to ping serial bus servo motors.",
    )

    # Create mutually-exclusive group of arguments to prevent conflicts
    group = parser.add_mutually_exclusive_group()
    group.add_argument("-a","--all", help="Ping all servos", action="store_true")
    group.add_argument("-i","--id", metavar="<servo_id>", help="Ping servo by ID", type=int)

    args = parser.parse_args()

    print(args)
    
    # match arg:
    #     case args.number:
    if args.all == True:
        num_servos = 50
        return ["all", num_servos]
    elif args.id != None:
        id = args.id
        return ["id", id]


def startConnection():
    with open("config.json", "r") as config_file:
        config = json.load(config_file)

    BAUDRATE = config["serial_settings"]["baudrate"]
    DEVICE = config["serial_settings"]["device"]

    portHandler = PortHandler(DEVICE)

    if portHandler.openPort():
        print(f"Port opened successfully: {DEVICE}")
    else:
        print("Failed to open the port")
        quit()
    
    if portHandler.setBaudRate(BAUDRATE):
        print(f"Baudrate set to {BAUDRATE} successfully")
    else:
        print("Failed to change the baudrate")
        quit()

    config_file.close()

    return portHandler


def requestData(ping_target):
    connection = startConnection()
    packetHandler = sms_sts(connection)
    single_success = False

    if ping_target[0] == "all":
        for id in range(ping_target[1]):
            scs_model_number, scs_comm_result, scs_error = packetHandler.ping(id)
            if scs_comm_result == 0:
                print(f"[ID:{id}] ping succeeded. STServo model number: {scs_model_number}")
                single_success = True
    elif ping_target[0] == "id":
        id = ping_target[1]
        scs_model_number, scs_comm_result, scs_error = packetHandler.ping(id)
        if scs_comm_result == 0:
            print(f"[ID:{id}] ping succeeded. STServo model number: {scs_model_number}")
            single_success = True

    
    if single_success == False:
        print(f"No motors were found.")
        
    connection.closePort()


def main():
    targets = parseArguments()

    if targets == None:
        print("Command was missing arguments.")
        exit()

    requestData(targets)


if __name__=="__main__":
    main()