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
    """Robot arm class for scripting motions.

    Attributes
    ----------
    joint_info: Dictionary of joint properties such as ID, angle, speed, etc.

    current_tf: Numpy array of the transformation matrix for the end effector's current pose.
    
    Methods
    -------
    ping(id): Ping a servo by ID.

    home(): Move the robot to the "home" position defined in the config.json.

    moveX(x): Move a distance x along the global x axis relative to the current position.

    moveY(y): Move a distance y along the global y axis relative to the current position.

    moveZ(z): Move a distance z along the global z axis relative to the current position.
    """

    def __init__(self):
        # Load kinematics library class
        self._k = Kinematics()

        # Set up connection to robot
        self._load_config()
        self._conn = self._startConnection()
        self._packetHandler = sms_sts(self._conn)

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

        
        self._activeServos = self._getActiveServos()     # Search for servo IDs and store them in a list
        self.current_tf = self.computeFK()               # Initialize kinematics
        self._move_complete = True                       # Initialize check properties

        # Wait a couple of seconds for all operations to complete
        print("Robot initialized successfully. Moving to HOME position.")

        self.home()     # Home the robot

    # =================
    # INTERNAL METHODS
    # =================

    def _load_config(self):
        """Set up class using configuration parameters.
        
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
        self._BAUDRATE = config["serial_settings"]["baudrate"]
        self._DEVICE = config["serial_settings"]["device"]

        # Load servo settings
        self._MAX_ID = config["servo_params"]["max_id"]
        self.SPEED = config["servo_params"]["default_speed"]
        self.ACCEL = config["servo_params"]["default_accel"]
        self.MIN_POS = config["servo_params"]["min_pos"]
        self.MAX_POS = config["servo_params"]["max_pos"]
        self.CENTER_POS = config["servo_params"]["center_pos"]
        self._DEADBAND = config["servo_params"]["deadband"]

        # Load running parameters
        self.MOVE_DELAY = config["run_params"]["move_delay"]

        # Load pre-defined positions
        self.HOME = config["defined_positions"]["home"]     # A list of angles in degrees

        # Close file when finished
        f.close()

    def _startConnection(self):
        """Begin serial communication with the robot.
        
        Serial communication parameters are defined in config.json. Ensure that the robot's "serial forwarding" setting is turned on.

        Returns
        -------
        The PortHandler object.
        """

        # Create an instance of the PortHandler object, which is basically the connection object.
        portHandler = PortHandler(self._DEVICE)

        if portHandler.openPort():
            print(f"Port opened successfully: {self._DEVICE}")

        else:
            print("Failed to open the port")
            exit()
        
        if portHandler.setBaudRate(self._BAUDRATE):
            print(f"Baudrate set to {self._BAUDRATE} successfully")
        else:
            print("Failed to change the baudrate")
            exit()

        return portHandler
    
    def _getActiveServos(self):
        """Fetches active servo IDs.

        Searches through all IDs from 0 to max_id for connected servo motors, and adds them to the class property joint_info.
        
        Returns
        -------
        A list of active servo IDs.
        """

        activeServos = []
        joint_idx = 1
        for id in range(self._MAX_ID):
            sts_model_number, sts_comm_result, sts_error = self._packetHandler.ping(id)
            if sts_comm_result == 0:
                print(f"Found active servo. ID: {id}.")
                activeServos.append(id)                         # Add servo ID to temporary list
                self.joint_info[joint_idx]["servo_id"] = id     # Add servo ID to joint_info dictionary

                joint_idx += 1
        
        return activeServos

    def _angleToServoPos(self, angle:float, unit:str="deg"):
        """Converts an input angle to a servo position value.
        
        Args
        ----
        angle: float
            An angle between -180 deg and 180 deg or -pi and pi.

        unit: str
            The unit of the angles. Set to degrees by default.

        Returns
        -------
        A servo position value.
        """

        # Normalize servo position values around the midpoint
        match unit:
            case "deg":
                pos_change = (angle / 180.0) * self.CENTER_POS
            case "rad":
                pos_change = (angle / (pi)) * self.CENTER_POS
            case _:
                print("Invalid unit.")

        pos = self.CENTER_POS + pos_change  # Handles both positive and negative input angles.

        return int(pos)
            
    def _servoPosToAngle(self, servo_pos:int, unit:str="deg"):
        """Converts an servo position to an angle.
        
        Args
        ----
        servo_pos: int
            An integer servo position value between the min and max specified in the config.json.

        unit: str
            The unit of the output angle. Set to degrees by default.

        Returns
        -------
        An angle value.
        """
        match unit:
            case "deg":
                return ((360.0 / 4095) * servo_pos) - 180
            case "rad":
                return ((2 * pi / 4095) * servo_pos) - pi
            case _:
                print("Invalid unit.")
        
    # ===============
    # UTILITY METHODS
    # ===============

    def ping(self, id:int):
        """Ping one or multiple servos.

        Args
        ----
        id: int
            Pass an integer ID from 1 to _MAX_ID to ping a particular servo. Passing and ID of 0 will ping all servos.
        """

        single_success = False

        match id:
            case 0:
                for id in self._activeServos:
                    sts_model_number, sts_comm_result, sts_error = self._packetHandler.ping(id)
                    if sts_comm_result == 0:
                        print(f"[ID:{id}] ping succeeded. STServo model number: {sts_model_number}")
                        single_success = True
            case _:
                sts_model_number, sts_comm_result, sts_error = self._packetHandler.ping(id)
                if sts_comm_result == 0:
                    print(f"[ID:{id}] ping succeeded. STServo model number: {sts_model_number}")
                    single_success = True

        if single_success == False:
            print(f"No motors were found.")

    # ============
    # READ METHODS
    # ============

    def _readServoPos(self, id:int):
        """Read the position of one or multiple servos.

        This method updates the servo_pos entries in the class attribute joint_info.

        Args
        ----
        id: int
            Pass an integer ID from 1 to _MAX_ID to read the position of a particular servo. Passing an id of 0 will read the positions of all servos.
        """

        match id:
            case 0:
                joint_idx = 1
                for id in self._activeServos:
                    sts_current_position, sts_comm_result, sts_error = self._packetHandler.ReadPos(id)
                    if sts_comm_result == 0:
                        self.joint_info[joint_idx]["servo_pos"] = sts_current_position
                        print(f"Servo [ID {id}] position: {sts_current_position}")
                    else:
                        print(f"Error reading servo with ID {id}: {sts_error}")
            case _:
                sts_current_position, sts_comm_result, sts_error = self._packetHandler.ReadPos(id)
                if sts_comm_result == 0:
                    print(f"Servo [ID {id}] position: {sts_current_position}")
                else:
                    print(f"Error reading servo with ID {id}: {sts_error}")
            
    def readJointAngle(self, id:int):
        """Reads joint angles in radians to be usable by other methods, and prints out the angles in degrees.

        This method updates the angle entries in the class attribute joint_info.
        
        Args
        ----
        id: int
            Pass an integer ID from 1 to 5 to read the angle of a particular joint. Passing an id of 0 will read the positions of all joints.

        TODO: Clean up variable names and logic. Try to use list comprehension.
        """
        match id:
            case 0:
                joint_angles = []
                joint_idx = 1
                for id in self._activeServos:
                    sts_current_position, sts_comm_result, sts_error = self._packetHandler.ReadPos(id)
                    if sts_comm_result == 0:
                        self.joint_info[joint_idx]["servo_pos"] = sts_current_position
                        angle = round(self._servoPosToAngle(sts_current_position, "rad"), 5)
                        self.joint_info[joint_idx]["angle"] = angle
                        joint_angles.append(angle)

                        joint_idx += 1

                print([degrees(joint_angle) for joint_angle in joint_angles])
            case _:
                sts_current_position, sts_comm_result, sts_error = self._packetHandler.ReadPos(id)
                if sts_comm_result == 0:

                    for i, key in enumerate(self.joint_info):
                        if key["id"]==id:
                            self.joint_info[key]["servo_pos"] = sts_current_position
                            angle = self._servoPosToAngle(sts_current_position)
                            self.joint_info[key]["angle"] = angle
                            print(f"Servo [ID {id}] angle: {angle}")

    def contReadJointAngle(self, id:int):
        """Continuously print out the joint angles in degrees.
        
        Args
        ----
        id: int
            Pass an integer ID from 1 to 5 to read the angle of a particular joint. Passing an id of 0 will read the positions of all joints.
        """
        while(True):
            self.readJointAngle(id)

    def checkIfMoving(self):
        """Check if robot is moving.

        Blocks execution of code until the _move_complete flag is set to True.
        """

        while self._move_complete==False:
            moving_checks = [None, None, None, None, None]

            for idx, servo_id in enumerate(self._activeServos):
                is_moving, sts_comm_result, sts_error = self._packetHandler.ReadMoving(servo_id)
                moving_checks[idx] = is_moving
            if 1 in moving_checks:
                self._move_complete = False
            else:
                self._move_complete = True
                print(f"movement complete")
                    
    # ======================
    # MANUAL CONTROL METHODS
    # ======================

    def disableServo(self, id:int):
        """Disable a servo or all servo.
        
        TODO: Before implementing this method, the packet info must be able to read the "enable" setting.
        """
        pass

    def writeServoPos(self, id:int, pos: int):
        """Write position to a servo.
        
        The servo will move to the position specified, and it will run at the default speed and acceleration.
        
        Args
        ----
        id: int
            Pass an integer ID from 1 to _MAX_ID to write a position to a particular servo. Passing an id of 0 will write to all servos.

        pos: int
            The target servo position value (absolute).
        """

        match id:
            case 0:
                for id in self._activeServos:
                    sts_comm_result, sts_error = self._packetHandler.WritePosEx(id, pos, self.SPEED, self.ACCEL)
                    if sts_comm_result == 0:
                        print(f"Servo [ID {id}] new position: {pos}")
                    else:
                        print(f"Error writing to servo with ID {id}: {sts_error}.")
            case _:
                sts_comm_result, sts_error = self._packetHandler.WritePosEx(id, pos, self.SPEED, self.ACCEL)
                if sts_comm_result == 0:
                    print(f"Servo [ID {id}] new position: {pos}")
                else:
                    print(f"Error writing to servo with ID {id}: {sts_error}.")

    def syncWriteServoPos(self, id_list:list, pos_list:list, speed_list:list=None, accel_list:list=None):
        """I don't really know what the low level function does.

        Some form of writing servo position. Maybe writing to all servos at once?

        Args
        ----
        id_list: list
            A list of servo IDs corresponding to each joint, ordered the same as the joint order.

        pos_list: list
            A list of servo position values to target, ordered the same as the joint order.

        speed_list: list
            A list of speed values, ordered the same as the joint order.

        accel_list:list
            A list of acceleration values, ordered the same as the joint order.

        TODO: Add individual servo speed/accel control. Try to understand what's going on in the low level function.
        """

        for (id, servo_pos) in zip(id_list, pos_list):
            # Add parameters to memory
            sts_addparam_result = self._packetHandler.SyncWritePosEx(id, servo_pos, self.SPEED, self.ACCEL)

            # Write the parameters that were in memory
            sts_comm_result = self._packetHandler.groupSyncWrite.txPacket()

            # Clear memory
            self._packetHandler.groupSyncWrite.clearParam()

    def writeAngle(self, id:int, angle:float):
        """Write an angle to a servo or all servos.

        Args
        ----
        id: int
            Pass an integer ID from 1 to _MAX_ID to write an angle to a particular servo. Passing an id of 0 will write to all servos.

        angle: float
            Target angle in degrees.

        TODO: Decide if support for radians is necessary for this method.
        """

        servo_pos = int(self._angleToServoPos(angle))

        match id:
            case 0:
                print(f"case 0")
                for id in self._activeServos:
                    sts_comm_result, sts_error = self._packetHandler.WritePosEx(id, servo_pos, self.SPEED, self.ACCEL)
                    if sts_comm_result == 0:
                        print(f"Servo [ID {id}] new angle: {angle}")
                        print(f"Servo [ID {id}] new position: {servo_pos}")
                    else:
                        print(f"Error writing to servo with ID {id}: {sts_error}.")
            case _:
                print(f"case _")
                sts_comm_result, sts_error = self._packetHandler.WritePosEx(id, servo_pos, self.SPEED, self.ACCEL)
                if sts_comm_result == 0:
                    print(f"Servo [ID {id}] new position: {servo_pos}")
                else:
                    print(f"Error writing to servo with ID {id}: {sts_error}.")

    def writeAllAngles(self, angles:list):
        """Writes angles to all servos. 
        
        Args
        ----
        angles:list
            A list of angles in radians.
        """

        # servo_positions = list(map(lambda angle: self._angleToServoPos(angle, "rad"), angles))
        servo_positions = [self._angleToServoPos(angle, "rad") for angle in angles]
        idx = 0

        for joint in self.joint_info.values():
            self.writeServoPos(joint["servo_id"], servo_positions[idx])
            idx += 1

    def syncWriteAngles(self, angles:list):
        """Sync write angles.

        Checks that all joints have stopped moving, then writes new target angles to them. Converts the input angles to servo positions
        then passes them to the syncWriteServoPos() method.

        Args
        ----
        angles:list
            A list of target angles.
        """

        self.checkIfMoving()

        servo_positions = [self._angleToServoPos(angle, "rad") for angle in angles]
        ids = [joint["servo_id"] for joint in self.joint_info.values()]

        self.syncWriteServoPos(ids, servo_positions)

        self._move_complete = False

    def setSpeed(self, speed:int):
        """Globally sets the speed. I don't think this is very useful right now."""

    
    def readEnableSetting(self, id:int):
        """Checks if the motor is enabled.

        Updates the joint_info attribute.
        
        Args
        ----
        id: int
            Pass an integer ID from 1 to _MAX_ID to check a particular servo. Passing an id of 0 will read from all servos.

        TODO: Implement an all servos call.
        """

        enabled = self._packetHandler.readEnable(id)

        return enabled

    # ===========
    # KINEMATICS
    # ===========

    def computeFK(self):
        """Calculate the transformation matrix of the current end effector pose.
        
        Returns
        -------
        A numpy array for the 4x4 matrix that represents the rotation and position of the end effector.
        """

        self.readJointAngle(0)

        joint_angles = [joint["angle"] for joint in self.joint_info.values()]   # List comprehension to extract angle values from dictionary

        FK = self._k.get_FK_mat(joint_angles)

        print(f"FK: {FK}")  # Debug

        return FK
    
    def getEEPos(self):
        """ Get the end effector's position in Cartesian coordinates relative to the base frame.
        
        Returns
        -------
        A numpy array for the 3x1 position vector.
        """

        fk_mat = self.computeFK()
        pos_vec = fk_mat[:3, 3].transpose()

        return pos_vec

    def getEERot(self):
        """Get the end effector's rotation matrix relative to the base frame.
        
        Returns
        -------
        A numpy array for the 3x3 rotation matrix.
        """

        fk_mat = self.computeFK()
        rot_mat = fk_mat[:3,:3]

        return rot_mat


    def computeIKFromPosition(self, pos_vec:list):
        """Compute the target joint angles given a target position vector [x, y, z].
        
        Args
        ----
        pos_vec: list
            A list of Cartesian coordinates (x, y, z) relative to the global origin.

        Returns
        -------
        A list of joint angles in radians.
        """

        current_frame = self.computeFK()
        target_frame = self._k.tf_from_position(pos_vec, current_frame)
        
        target_joint_angles = self._k.calcAllJointAngles(target_frame)

        return target_joint_angles


    def getJointAnglesFromTF(self):
        """Get the joint angles from the end effector's transformation matrix.

        Returns
        -------
        a list of angles in degrees.
        
        NOTE: Is this method even useful?
        """
        fk = self.computeFK()
        joint_angles = self._k.calc_joint_angles(fk)

        deg_angles = [angle for angle in joint_angles]

        return deg_angles

    # =============
    # BASIC MOTIONS
    # =============
    
    def moveX(self, x, delay=None):
        """Linear move in the x direction.
        
        Args
        ----
        x: float
            A distance in mm along the global x axis.

        delay: float
            Delay in ms after the move completes.

        TODO: Properly implement the delay in the library.
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
        
        Args
        ----
        y: float
            A distance in mm along the global y axis.

        delay: float
            Delay in ms after the move completes.

        TODO: Properly implement the delay in the library.
        """

        if delay==None:
            delay = self.MOVE_DELAY

        target_joint_angles = self.computeIKFromPosition([0, y, 0])
        self.syncWriteAngles(target_joint_angles)
        time.sleep(delay)

        return self
    

    def moveZ(self, z, delay=None):
        """Linear move in the z direction.
        
        Args
        ----
        x: float
            A distance in mm along the global z axis.

        delay: float
            Delay in ms after the move completes.

        TODO: Properly implement the delay in the library.    
        """

        if delay==None:
            delay = self.MOVE_DELAY

        target_joint_angles = self.computeIKFromPosition([0, 0, z])
        self.syncWriteAngles(target_joint_angles)
        time.sleep(delay)

        return self
    

    def home(self, delay=None):
        """Go to the home position defined in config.json.
        
        This method sets a manual target angle to each servo, so the arm must be calibrated beforehand.
        
        Args
        ----
        delay: float
            Delay in ms after the move completes.

        TODO: Properly implement the delay in the library.
        """

        if delay==None:
            delay = self.MOVE_DELAY

        targets_in_radians = [radians(angle) for angle in self.HOME]
        self.syncWriteAngles(targets_in_radians)
        time.sleep(delay)

        return self