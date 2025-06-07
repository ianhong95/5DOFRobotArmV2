import serial
import time
import readline
import sys
import argparse
import json
import os


# sys.path hack to import a module from a parent directory. This basically adds the parent directory to PATH.
sys.path.append("..")
from robot_class import RobotArm

sys.path.append("../scservo_sdk")


def setupReadline():
    readline.parse_and_bind("tab: complete")  # Optional autocomplete
    try:
        readline.read_history_file(".printer_history")  # Load previous session history
    except FileNotFoundError:
        print("Error: file not found")


def runTerminal():
    try:
        # Change working directory to parent directory so that the robot library can find the config.json
        os.chdir("..")
        r = RobotArm()
    except:
        print("Error connecting to robot")

    time.sleep(1)

    while True:
        cmd = input("> ")
        try:
            result = eval(cmd)
            # print(f"> Executed {cmd}")
        except ValueError:
            print("Invalid argument")
        except:
            print("Invalid command.")
        print(f"--------------------------------")


def main():
    setupReadline()
    runTerminal()


if __name__=="__main__":
    main()