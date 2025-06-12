"""
This is a library for the RobotArm class that contains all the attributes and methods to control the arm.

TODO:
- Add more debugging methods
- Clean up hacky code
- Continuously read position?
- Implement deadband?
- Add a method to read and save current position to config.json or to an attribute
- Set wrist or EE orientation
"""

# Standard library imports
import json
import logging
import time
from math import pi, degrees, radians

# 3rd party library imports
import numpy as np

# Custom library imports
from scservo_sdk import *   # Uses SC Servo SDK library
from kinematics import Kinematics

# Initialize logger
logger = logging.getLogger(__name__)
logging.basicConfig(filename="api_logs.log", level=logging.INFO)


class RobotArm:
    def __init__(self):
        # Load kinematics library class
        self.k = Kinematics()

        # Set up connection to robot
        self._load_config()
        self.conn = self._startConnection()
        self.packetHandler = sms_sts(self.conn)

        # Initialize joint info dictionary
        self.joint_info = {
            1: {
                "servo_id": 0,
                "servo_pos": 0,
                "angle": 0,
                "speed": self.SPEED,
                "accel": self.ACCEL,
                "enabled": 0
            },
            2: {
                "servo_id": 0,
                "servo_pos": 0,
                "angle": 0,
                "speed": self.SPEED,
                "accel": self.ACCEL,
                "enabled": 0
            },
            3: {
                "servo_id": 0,
                "servo_pos": 0,
                "angle": 0,
                "speed": self.SPEED,
                "accel": self.ACCEL,
                "enabled": 0
            },
            4: {
                "servo_id": 0,
                "servo_pos": 0,
                "angle": 0,
                "speed": self.SPEED+250,
                "accel": self.ACCEL,
                "enabled": 0
            },
            5: {
                "servo_id": 0,
                "servo_pos": 0,
                "angle": 0,
                "speed": self.SPEED+500,
                "accel": self.ACCEL,
                "enabled": 0
            },                                
        }

        
        self.activeServos = self._getActiveServos()     # Search for servo IDs and store them in a list
        self.current_tf = self.computeFK()              # Initialize kinematics
        self.move_complete = True                       # Initialize check properties

        # Wait a couple of seconds for all operations to complete
        print("Robot initialized successfully. Moving to HOME position.")

        self.home()     # Home the robot


    """ INTERNAL METHODS """

    def _load_config(self):
        """
        Load configuration settings from a config.json file, and initialize them as properties for the class.
        """

        # Load the config file
        config_file = "config.json"
        try:
            with open(config_file, "r") as f:
                config = json.load(f)
        except:
            print(f"{config_file} not found.")
            exit()

        # Load serial connection settings
        self.BAUDRATE = config["serial_settings"]["baudrate"]
        self.DEVICE = config["serial_settings"]["device"]

        # Load servo settings
        self.MAX_ID = config["servo_params"]["max_id"]
        self.SPEED = config["servo_params"]["default_speed"]
        self.ACCEL = config["servo_params"]["default_accel"]
        self.MIN_POS = config["servo_params"]["min_pos"]
        self.MAX_POS = config["servo_params"]["max_pos"]
        self.CENTER_POS = config["servo_params"]["center_pos"]
        self.DEADBAND = config["servo_params"]["deadband"]

        # Load running parameters
        self.MOVE_DELAY = config["run_params"]["move_delay"]

        # Load pre-defined positions
        self.HOME = config["defined_positions"]["home"]     # A list of angles in degrees

        # Close file when finished
        f.close()


    def _startConnection(self):
        """
        Begin serial communication with the robot. Ensure that the robot's "serial forwarding" setting is turned on.
        """

        # Create an instance of the PortHandler object, which is basically the connection object.
        portHandler = PortHandler(self.DEVICE)

        if portHandler.openPort():
            print(f"Port opened successfully: {self.DEVICE}")

        else:
            print("Failed to open the port")
            exit()
        
        if portHandler.setBaudRate(self.BAUDRATE):
            print(f"Baudrate set to {self.BAUDRATE} successfully")
        else:
            print("Failed to change the baudrate")
            exit()

        return portHandler
    

    def _getActiveServos(self):
        """
        Searches through all IDs from 0 to max_id for connected servo motors, and adds them to the class property joint_info.
        
        Returns a list of the IDs.
        """
        activeServos = []
        joint_idx = 1
        for id in range(self.MAX_ID):
            sts_model_number, sts_comm_result, sts_error = self.packetHandler.ping(id)
            if sts_comm_result == 0:
                print(f"Found active servo. ID: {id}.")
                activeServos.append(id)                         # Add servo ID to temporary list
                self.joint_info[joint_idx]["servo_id"] = id     # Add servo ID to joint_info dictionary

                joint_idx += 1
        
        return activeServos


    def _angleToServoPos(self, angle:float, unit:str="deg"):
        """
        Converts an input angle (from -180 degrees to 180 degrees) to a servo position value. By default, the input angle is assumed to be in degrees, but you can specify "rad" to change it to radians.
        
        Returns a servo position value from min_pos to max_pos.
        """

        # Normalize servo position values around the midpoint
        match unit:
            case "deg":
                pos_change = (angle / 180.0) * self.CENTER_POS
            case "rad":
                pos_change = (angle / (pi)) * self.CENTER_POS
            case _:
                print("Invalid unit.")

        if angle >= 0:
            pos = self.CENTER_POS + pos_change
        elif angle < 0:
            pos = self.CENTER_POS + pos_change

        return int(pos)
            

    def _servoPosToAngle(self, servo_pos:float, unit:str="deg"):
        """
        Converts an servo position value to an angle.

        Returns an angle in degrees by default, but you can specify "rad" to change it to radians.
        """
        match unit:
            case "deg":
                return ((360.0 / 4095) * servo_pos) - 180
            case "rad":
                return ((2 * pi / 4095) * servo_pos) - pi
            case _:
                print("Invalid unit.")
        

    """ UTILITY METHODS """

    def ping(self, id:int):
        """
        Ping one or multiple servos.
        
        Pass an id integer from 1 to MAX_ID to ping a particular servo.
        Passing an id of 0 will ping all servos.
        """
        single_success = False

        match id:
            case 0:
                for id in self.activeServos:
                    sts_model_number, sts_comm_result, sts_error = self.packetHandler.ping(id)
                    if sts_comm_result == 0:
                        print(f"[ID:{id}] ping succeeded. STServo model number: {sts_model_number}")
                        single_success = True
            case _:
                sts_model_number, sts_comm_result, sts_error = self.packetHandler.ping(id)
                if sts_comm_result == 0:
                    print(f"[ID:{id}] ping succeeded. STServo model number: {sts_model_number}")
                    single_success = True

        if single_success == False:
            print(f"No motors were found.")


    """ READ METHODS """

    def readServoPos(self, id:int):
        """
        Read the position of one or multiple servos.
        
        Pass an id integer from 1 to MAX_ID to read the position of a particular servo.
        Passing an id of 0 will read the positions of all servos.
        """

        match id:
            case 0:
                joint_idx = 1
                for id in self.activeServos:
                    sts_current_position, sts_comm_result, sts_error = self.packetHandler.ReadPos(id)
                    if sts_comm_result == 0:
                        self.joint_info[joint_idx]["servo_pos"] = sts_current_position
                        print(f"Servo [ID {id}] position: {sts_current_position}")
                    else:
                        print(f"Error reading servo with ID {id}: {sts_error}")
            case _:
                sts_current_position, sts_comm_result, sts_error = self.packetHandler.ReadPos(id)
                if sts_comm_result == 0:
                    print(f"Servo [ID {id}] position: {sts_current_position}")
                else:
                    print(f"Error reading servo with ID {id}: {sts_error}")
            

    def readJointAngle(self, id:int):
        """
        Reads joint angles in radians to be usable by other methods, and prints out the angles in degrees.
        
        Takes a servo ID as an argument, and passing 0 will read all servo angles.

        TODO: Clean up variable names and logic. Try to use list comprehension.
        """
        match id:
            case 0:
                joint_angles = []
                joint_idx = 1
                for id in self.activeServos:
                    sts_current_position, sts_comm_result, sts_error = self.packetHandler.ReadPos(id)
                    if sts_comm_result == 0:
                        self.joint_info[joint_idx]["servo_pos"] = sts_current_position
                        angle = round(self._servoPosToAngle(sts_current_position, "rad"), 5)
                        self.joint_info[joint_idx]["angle"] = angle
                        joint_angles.append(angle)

                        joint_idx += 1

                print([degrees(joint_angle) for joint_angle in joint_angles])
            case _:
                sts_current_position, sts_comm_result, sts_error = self.packetHandler.ReadPos(id)
                if sts_comm_result == 0:

                    for i, key in enumerate(self.joint_info):
                        if key["id"]==id:
                            self.joint_info[key]["servo_pos"] = sts_current_position
                            angle = self._servoPosToAngle(sts_current_position)
                            self.joint_info[key]["angle"] = angle
                            print(f"Servo [ID {id}] angle: {angle}")



    def contReadJointAngle(self, id:int):
        """
        Continuously print out the joint angles in degrees.
        
        Takes a joint ID as an argument, and passing 0 will read all joints.
        """
        while(True):
            self.readJointAngle(id)


    def checkIfMoving(self):
        """
        Blocks execution of code until the move_complete flag is set to True.
        """

        while self.move_complete==False:
            moving_checks = [None, None, None, None, None]

            for idx, servo_id in enumerate(self.activeServos):
                is_moving, sts_comm_result, sts_error = self.packetHandler.ReadMoving(servo_id)
                moving_checks[idx] = is_moving
            if 1 in moving_checks:
                self.move_complete = False
            else:
                self.move_complete = True
                print(f"movement complete")
                    

    """ MANUAL SERVO CONTROL METHODS """

    def disableServo(self, id:int):
        pass


    def writeServoPos(self, id:int, pos: int):
        """
        Write position to a servo, and it will run at the default speed and acceleration.
        
        Pass an id integer from 1 to MAX_ID to write a position to a particular servo.
        Passing an id of 0 will write to all servos.
        """

        match id:
            case 0:
                for id in self.activeServos:
                    sts_comm_result, sts_error = self.packetHandler.WritePosEx(id, pos, self.SPEED, self.ACCEL)
                    if sts_comm_result == 0:
                        print(f"Servo [ID {id}] new position: {pos}")
                    else:
                        print(f"Error writing to servo with ID {id}: {sts_error}.")
            case _:
                sts_comm_result, sts_error = self.packetHandler.WritePosEx(id, pos, self.SPEED, self.ACCEL)
                if sts_comm_result == 0:
                    print(f"Servo [ID {id}] new position: {pos}")
                else:
                    print(f"Error writing to servo with ID {id}: {sts_error}.")


    def syncWriteServoPos(self, id_list:list, pos_list:list, speed_list:list=None, accel_list:list=None):
        """
        I don't really know what the low level function does.

        TODO: Add individual servo speed/accel control.
        """

        for (id, servo_pos) in zip(id_list, pos_list):
            # Add parameters to memory
            sts_addparam_result = self.packetHandler.SyncWritePosEx(id, servo_pos, self.SPEED, self.ACCEL)

            # Write the parameters that were in memory
            sts_comm_result = self.packetHandler.groupSyncWrite.txPacket()

            # Clear memory
            self.packetHandler.groupSyncWrite.clearParam()



    def writeAngle(self, id:int, angle:float):
        """
        Write an angle to a servo or all servos. The input angle is in degrees.
        """

        servo_pos = int(self._angleToServoPos(angle))

        match id:
            case 0:
                print(f"case 0")
                for id in self.activeServos:
                    sts_comm_result, sts_error = self.packetHandler.WritePosEx(id, servo_pos, self.SPEED, self.ACCEL)
                    if sts_comm_result == 0:
                        print(f"Servo [ID {id}] new angle: {angle}")
                        print(f"Servo [ID {id}] new position: {servo_pos}")
                    else:
                        print(f"Error writing to servo with ID {id}: {sts_error}.")
            case _:
                print(f"case _")
                sts_comm_result, sts_error = self.packetHandler.WritePosEx(id, servo_pos, self.SPEED, self.ACCEL)
                if sts_comm_result == 0:
                    print(f"Servo [ID {id}] new position: {servo_pos}")
                else:
                    print(f"Error writing to servo with ID {id}: {sts_error}.")


    def writeAllAngles(self, angles:list):
        """
        Writes angles to all servos. Takes a list of angles (radians) as an argument.
        """

        # servo_positions = list(map(lambda angle: self._angleToServoPos(angle, "rad"), angles))
        servo_positions = [self._angleToServoPos(angle, "rad") for angle in angles]
        idx = 0

        for joint in self.joint_info.values():
            self.writeServoPos(joint["servo_id"], servo_positions[idx])
            idx += 1


    def syncWriteAngles(self, angles:list):
        """
        Sync write?
        """
        self.checkIfMoving()

        servo_positions = [self._angleToServoPos(angle, "rad") for angle in angles]
        ids = [joint["servo_id"] for joint in self.joint_info.values()]

        self.syncWriteServoPos(ids, servo_positions)

        self.move_complete = False


    def setSpeed(self, speed:int):
        """
        Globally sets the speed. I don't think this is very useful right now.
        """

    
    def readEnableSetting(self, id:int):
        print(self.packetHandler.readEnable(id))


    """ Kinematics """
    def computeFK(self):
        """
        Compute the forward kinematics by reading the current joint angles.
        
        Returns a 4x4 matrix that represents the rotation and position of the end effector.
        """
        self.readJointAngle(0)

        joint_angles = [joint["angle"] for joint in self.joint_info.values()]   # List comprehension to extract angle values from dictionary

        FK = self.k.get_FK_mat(joint_angles)

        print(f"FK: {FK}")

        return FK
    

    def getEEPos(self):
        """
        Get the end effector's position in Cartesian coordinates relative to the base frame.
        
        Returns a 3x1 position vector.
        """
        fk_mat = self.computeFK()
        pos_vec = fk_mat[:3, 3].transpose()

        return pos_vec


    def getEERot(self):
        """
        Get the end effector's rotation matrix relative to the base frame.
        
        Returns a 3x3 rotation matrix.
        """
        fk_mat = self.computeFK()
        rot_mat = fk_mat[:3,:3]

        return rot_mat


    def computeIKFromPosition(self, pos_vec:list):
        """
        Compute the inverse kinematics given a position vector [x, y, z].\n
        
        Returns a list of joint angles.
        """
        current_frame = self.computeFK()
        target_frame = self.k.tf_from_position(pos_vec, current_frame)
        
        target_joint_angles = self.k.calcAllJointAngles(target_frame)

        return target_joint_angles


    def getJointAnglesFromTF(self):
        fk = self.computeFK()
        joint_angles = self.k.calc_joint_angles(fk)

        deg_angles = [angle for angle in joint_angles]

        print(deg_angles)


    """
    BASIC MOTIONS
    
    """
    
    def moveX(self, x, delay=None):
        """
        Linear move in the x direction.
        
        Pass an argument for the distance to move in mm.
        """
        if delay==None:
            delay = self.MOVE_DELAY

        target_joint_angles = self.computeIKFromPosition([x, 0, 0])
        self.syncWriteAngles(target_joint_angles)
        time.sleep(delay)

        return self


    def moveY(self, y, delay=None):
        """
        Linear move in the y direction.
        
        Pass an argument for the distance to move in mm.
        """
        if delay==None:
            delay = self.MOVE_DELAY

        target_joint_angles = self.computeIKFromPosition([0, y, 0])
        self.syncWriteAngles(target_joint_angles)
        time.sleep(delay)

        return self
    

    def moveZ(self, z, delay=None):
        """
        Linear move in the z direction.
        
        Pass an argument for the distance to move in mm.        
        """
        if delay==None:
            delay = self.MOVE_DELAY

        target_joint_angles = self.computeIKFromPosition([0, 0, z])
        self.syncWriteAngles(target_joint_angles)
        time.sleep(delay)

        return self
    

    def home(self, delay=None):
        """
        Go to the home position defined in config.json. This method sets a manual target angle to each servo.
        """

        if delay==None:
            delay = self.MOVE_DELAY

        targets_in_radians = [radians(angle) for angle in self.HOME]
        self.syncWriteAngles(targets_in_radians)
        time.sleep(delay)

        return self