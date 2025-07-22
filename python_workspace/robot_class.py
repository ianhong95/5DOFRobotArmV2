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

    move_x(x): Move a distance x along the global x axis relative to the current position.

    move_y(y): Move a distance y along the global y axis relative to the current position.

    move_z(z): Move a distance z along the global z axis relative to the current position.

    disable_servo(id): Disable the servo with the specified ID. Passing an ID of 0 will disable all servos.
    """

    def __init__(self):
        # Load kinematics library class
        self._k = Kinematics("config.json")

        # Set up connection to robot
        self._load_config()
        self._conn = self._start_connection()
        self._packetHandler = sms_sts(self._conn)

        self._activeServos = self._get_active_servos()  # Search for servo IDs and store them in a list
        
        self.current_tf = self.compute_fk() # Initialize kinematics

        self._move_complete = True  # Initialize check properties

        self.teach_positions = {}

        # Wait a couple of seconds for all operations to complete
        # print("Robot initialized successfully. Moving to HOME position.")

        # self.home() # Home the robot

    # =================
    # INTERNAL METHODS
    # =================

    def _load_config(self):
        """Set up class using configuration parameters.
        
        Load configuration settings from a config.json file, and initialize them as properties for the class. This method
        also initializes the joint_info dictionary.
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
        self.speed = config["servo_params"]["default_speed"]
        self.accel = config["servo_params"]["default_accel"]
        self.MIN_POS = config["servo_params"]["min_pos"]
        self.MAX_POS = config["servo_params"]["max_pos"]
        self.CENTER_POS = config["servo_params"]["center_pos"]
        self._DEADBAND = config["servo_params"]["deadband"]

        # Initialize joint info dictionary
        self.joint_info = {
            1: {
                "servo_id": 0,
                "servo_pos": 0,
                "angle": 0,
                "speed": self.speed,
                "accel": self.accel,
                "min_angle": radians(config["joint_limits"]["joint_1"]["min_angle"]),
                "max_angle": radians(config["joint_limits"]["joint_1"]["max_angle"]),
                "enabled": 0
            },
            2: {
                "servo_id": 0,
                "servo_pos": 0,
                "angle": 0,
                "speed": self.speed,
                "accel": self.accel,
                "min_angle": radians(config["joint_limits"]["joint_2"]["min_angle"]),
                "max_angle": radians(config["joint_limits"]["joint_2"]["max_angle"]),                
                "enabled": 0
            },
            3: {
                "servo_id": 0,
                "servo_pos": 0,
                "angle": 0,
                "speed": self.speed,
                "accel": self.accel,
                "min_angle": radians(config["joint_limits"]["joint_3"]["min_angle"]),
                "max_angle": radians(config["joint_limits"]["joint_3"]["max_angle"]),                
                "enabled": 0
            },
            4: {
                "servo_id": 0,
                "servo_pos": 0,
                "angle": 0,
                "speed": self.speed+250,
                "accel": self.accel,
                "min_angle": radians(config["joint_limits"]["joint_4"]["min_angle"]),
                "max_angle": radians(config["joint_limits"]["joint_4"]["max_angle"]),                
                "enabled": 0
            },
            5: {
                "servo_id": 0,
                "servo_pos": 0,
                "angle": 0,
                "speed": self.speed+500,
                "accel": self.accel,
                "min_angle": radians(config["joint_limits"]["joint_5"]["min_angle"]),
                "max_angle": radians(config["joint_limits"]["joint_5"]["max_angle"]),                
                "enabled": 0
            },                                
        }

        # Load running parameters
        self.MOVE_DELAY = config["run_params"]["move_delay"]

        # Load pre-defined positions
        self.saved_positions = {
            0: {
                "alias": "HOME",
                "joint_angles": config["defined_positions"]["home"],     # A list of angles in degrees
                "ee_position": [],
                "ee_orientation": []
            }
        }

        # Close file when finished
        f.close()

    def _start_connection(self):
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
    
    def _get_active_servos(self):
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
                br = self._packetHandler.readBaudrate(id)
                print(f"Found active servo. ID: {id}. Baudrate is {br}")
                activeServos.append(id)                         # Add servo ID to temporary list
                self.joint_info[joint_idx]["servo_id"] = id     # Add servo ID to joint_info dictionary

                joint_idx += 1

        if len(activeServos) < 5:
            print("Not enough servos found. Could not initialize robot.")
            exit()
        
        return activeServos

    def _angle_to_servo_pos(self, angle:float, unit:str="deg"):
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
            
    def _servo_pos_to_angle(self, servo_pos:int, unit:str="deg"):
        """Converts an servo position to an angle.
        
        Args
        ----
        servo_pos: int
            An integer servo position value between the min and max specified in the config.json.

        unit: str
            The unit of the output angle. Set to degrees by default.

        Returns
        -------
        An angle value (degrees by default).
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

    def _read_servo_pos(self, id:int):
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
            
    def read_joint_angle(self, id:int):
        """Reads joint angles in radians to be usable by other methods, and prints out the angles in degrees.

        This method updates the angle entries in the class attribute joint_info.
        
        Args
        ----
        id: int
            Pass an integer ID from 1 to 5 to read the angle of a particular joint. Passing an id of 0 will read the positions of all joints.

        Returns
        -------
        Either a list of 5 angles in radians or a single angle as a float.

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
                        angle = round(self._servo_pos_to_angle(sts_current_position, "rad"), 5)
                        self.joint_info[joint_idx]["angle"] = angle
                        joint_angles.append(angle)

                        joint_idx += 1
                    else:
                        print(f"Failed to access servo {id}. Cancelling motion.")
                        break
                
                return joint_angles
            case _:
                sts_current_position, sts_comm_result, sts_error = self._packetHandler.ReadPos(id)
                if sts_comm_result == 0:
                    for i, key in enumerate(self.joint_info):
                        if key["id"]==id:
                            self.joint_info[key]["servo_pos"] = sts_current_position
                            angle = self._servo_pos_to_angle(sts_current_position)
                            self.joint_info[key]["angle"] = angle
                            print(f"Servo [ID {id}] angle: {angle}")

                            return angle

    def cont_read_joint_angle(self, id:int):
        """Continuously print out the joint angles in degrees.
        
        Args
        ----
        id: int
            Pass an integer ID from 1 to 5 to read the angle of a particular joint. Passing an id of 0 will read the positions of all joints.
        """
        while(True):
            self.read_joint_angle(id)

    # ========
    # TEACHING
    # ========

    def set_teach_entry(self, id: int, alias: str = ""):
        """Save the robot's current position into a dictionary.

        Args
        ----
        alias: str
            A unique alias for the saved position to reference it later.
        """
        current_angles = [degrees(joint["angle"]) for joint in self.joint_info.values()]
    
        current_tf = self.compute_fk()  # read_joint_angles() is called inside this method
        pos_vec = current_tf[:3, 3].transpose()

        current_angles = [degrees(joint["angle"]) for joint in self.joint_info.values()]

        self.saved_positions[id] = {
            "alias": alias,
            "joint_angles": current_angles,
            "ee_position": pos_vec
        }

        return [id, alias, current_angles, pos_vec]

    def save_new_position(self, alias: str = ""):
        """Save the robot's current position as a new entry in the saved_positions dictionary."""

        for idx in range(len(self.saved_positions) + 1):
            if idx not in [id for id in self.saved_positions.keys()]:
                new_id = idx
                break

        self.set_teach_entry(new_id, alias)

    def update_position_by_id(self, entry_id: int):
        """Update an entry in the saved_positions dictionary by passing an ID."""

        if entry_id in self.saved_positions.keys(): # Check if id exists
            self.set_teach_entry(entry_id)
        else:
            print("ID {entry_id} does not exist. Please create a new entry using save_new_position().")

    def update_position_by_alias(self, alias: str):
        """Update an entry in the saved_positions dictionary by passing an alias."""

        for id in self.saved_positions.keys():
            if self.saved_positions[id]["alias"] == alias:
                self.set_teach_entry(id)
                break

    def list_saved_positions(self):
        """Prints out the dictionary of saved positions."""
        print(self.saved_positions)

    def recall_by_id(self, id: int):
        target_angles = [radians(angle) for angle in self.saved_positions[id]["joint_angles"]]

        print(target_angles)

        self.sync_write_angles(target_angles)
        
        return self

    def recall_by_name(self, name: str):
        """Move to a saved position."""

        target_angles = [radians(angle) for angle in self.saved_positions[name]["joint_angles"]]

        self.sync_write_angles(target_angles)

    def recall_all(self):
        for entry_id in self.saved_positions.keys():
            self.recall_by_id(entry_id)

    # ============
    # TOGGLE MODES
    # ============

    def set_teach_mode(self, on: bool):
        """
        """
        self.disable_servo(0)

        # alias = input("Enter a name for the position (leave blank to set it to an auto-generated ID): ")
        # self.save_new_position(alias)


    # ==================
    # VALIDATION METHODS
    # ==================

    def _check_joint_limits(self, joint_idx, target_angle):
        """Check if the target angle is within the joint's limits.
        
        Args
        ----
        joint_idx: int
            The joint's index.

        target_angle: float
            The target angle in radians.

        Returns
        -------
        is_valid: boolean
            Whether the angle is within the limits or not.
        """

        if (target_angle >= self.joint_info[joint_idx]["min_angle"] and
            target_angle <= self.joint_info[joint_idx]["max_angle"]
            ):
            return True
        else:
            return False            

    def _check_if_moving(self):
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

    def disable_servo(self, id:int):
        """Disable a servo or all servos.

        Sets the servo to not hold torque. It is automatically re-enabled when the servo is commanded to a position.

        Args
        ----
        id: int
            Pass the ID of the servo to disable, or pass an ID of 0 to disable all servos.
        """
        match id:
            case 0:
                for id in self._activeServos:
                    sts_comm_result, sts_error = self._packetHandler.writeEnable(id, 0)
                    if sts_comm_result == 0:
                        print(f"Servo [ID {id}] disabled")
                    else:
                        print(f"Error writing to servo with ID {id}: {sts_error}.")
            case _:
                sts_comm_result, sts_error = self._packetHandler.writeEnable(id, 0)
                if sts_comm_result == 0:
                    print(f"Servo [ID {id}] disabled")
                else:
                    print(f"Error writing to servo with ID {id}: {sts_error}.")

    def write_servo_pos(self, id:int, pos: int):
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
                    sts_comm_result, sts_error = self._packetHandler.WritePosEx(id, pos, self.speed, self.accel)
                    if sts_comm_result == 0:
                        print(f"Servo [ID {id}] new position: {pos}")
                    else:
                        print(f"Error writing to servo with ID {id}: {sts_error}.")
            case _:
                sts_comm_result, sts_error = self._packetHandler.WritePosEx(id, pos, self.speed, self.accel)
                if sts_comm_result == 0:
                    print(f"Servo [ID {id}] new position: {pos}")
                else:
                    print(f"Error writing to servo with ID {id}: {sts_error}.")

    def sync_write_servo_pos(self, id_list:list, pos_list:list, speed_list:list=None, accel_list:list=None):
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
            sts_addparam_result = self._packetHandler.SyncWritePosEx(id, servo_pos, self.speed, self.accel)

            # Write the parameters that were in memory
            sts_comm_result = self._packetHandler.groupSyncWrite.txPacket()

            # Clear memory
            self._packetHandler.groupSyncWrite.clearParam()

    def write_angle(self, id:int, angle:float):
        """Write an angle to a servo or all servos.

        This method should only be used for manual servo control.

        Args
        ----
        id: int
            Pass an integer ID from 1 to _MAX_ID to write an angle to a particular servo. Passing an id of 0 will write to all servos.

        angle: float
            Target angle in degrees.

        TODO: Decide if support for radians is necessary for this method.
        """

        servo_pos = int(self._angle_to_servo_pos(angle))

        match id:
            case 0:
                print(f"case 0")
                for id in self._activeServos:
                    sts_comm_result, sts_error = self._packetHandler.WritePosEx(id, servo_pos, self.speed, self.accel)
                    if sts_comm_result == 0:
                        print(f"Servo [ID {id}] new angle: {angle}")
                        print(f"Servo [ID {id}] new position: {servo_pos}")
                    else:
                        print(f"Error writing to servo with ID {id}: {sts_error}.")
            case _:
                print(f"case _")
                sts_comm_result, sts_error = self._packetHandler.WritePosEx(id, servo_pos, self.speed, self.accel)
                if sts_comm_result == 0:
                    print(f"Servo [ID {id}] new position: {servo_pos}")
                else:
                    print(f"Error writing to servo with ID {id}: {sts_error}.")

    def write_all_angles(self, angles:list):
        """Writes angles to all servos. 
        
        Args
        ----
        angles:list
            A list of angles in radians.
        """

        # servo_positions = list(map(lambda angle: self._angle_to_servo_pos(angle, "rad"), angles))
        servo_positions = [self._angle_to_servo_pos(angle, "rad") for angle in angles]
        idx = 0

        for joint in self.joint_info.values():
            self.write_servo_pos(joint["servo_id"], servo_positions[idx])
            idx += 1

    def sync_write_angles(self, angles:list):
        """Sync write angles.

        Checks that all joints have stopped moving, then writes new target angles to them. Converts the input angles to servo positions
        then passes them to the sync_write_servo_pos() method.

        Args
        ----
        angles:list
            A list of target angles in radians.
        """

        self._check_if_moving()

        servo_positions = [self._angle_to_servo_pos(angle, "rad") for angle in angles]
        ids = [joint["servo_id"] for joint in self.joint_info.values()]

        self.sync_write_servo_pos(ids, servo_positions)

        self._move_complete = False

    def set_speed(self, speed:int):
        """Globally sets the speed. I don't think this is very useful right now."""

    
    def read_enable(self, id:int):
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

    def compute_fk(self):
        """Calculate the transformation matrix of the current end effector pose.
        
        Returns
        -------
        A numpy array for the 4x4 matrix that represents the rotation and position of the end effector.
        """

        self.read_joint_angle(0)

        joint_angles = [joint["angle"] for joint in self.joint_info.values()]   # List comprehension to extract angle values from dictionary

        FK = self._k.calc_fk_mat(joint_angles)

        return FK
    
    def get_ee_pos(self):
        """ Get the end effector's position in Cartesian coordinates relative to the base frame.
        
        Returns
        -------
        A numpy array for the 3x1 position vector.
        """

        fk_mat = self.compute_fk()
        pos_vec = fk_mat[:3, 3].transpose()

        return pos_vec

    def get_ee_rot(self):
        """Get the end effector's rotation matrix relative to the base frame.
        
        Returns
        -------
        A numpy array for the 3x3 rotation matrix.
        """

        fk_mat = self.compute_fk()
        rot_mat = fk_mat[:3,:3]

        return rot_mat


    def compute_ik_from_pos(self, pos_vec:list):
        """Compute the target joint angles given a target position vector [x, y, z].
        
        Args
        ----
        pos_vec: list
            A list of Cartesian coordinates (x, y, z) relative to the global origin.

        Returns
        -------
        A list of joint angles in radians.
        """

        current_frame = self.compute_fk()

        try:
            target_frame = self._k.calc_tf_from_position(pos_vec, current_frame)
        except Exception as e:
            print(e)
        
        target_joint_angles = self._k.calc_all_joint_angles(target_frame)

        return target_joint_angles


    def get_joint_angles_from_tf(self):
        """Get the joint angles from the end effector's transformation matrix.

        Returns
        -------
        a list of angles in degrees.
        
        NOTE: Is this method even useful?
        """
        fk = self.compute_fk()
        joint_angles = self._k.calc_joint_angles(fk)

        deg_angles = [angle for angle in joint_angles]

        return deg_angles

    # =============
    # BASIC MOTIONS
    # =============
    
    def move_x(self, x: float, delay=None):
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

        target_joint_angles = self.compute_ik_from_pos([x, 0, 0])
        self.sync_write_angles(target_joint_angles)
        time.sleep(delay)

        return self


    def move_y(self, y: float, delay=None):
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

        target_joint_angles = self.compute_ik_from_pos([0, y, 0])
        self.sync_write_angles(target_joint_angles)
        time.sleep(delay)

        return self
    

    def move_z(self, z: float, delay=None):
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

        target_joint_angles = self.compute_ik_from_pos([0, 0, z])
        self.sync_write_angles(target_joint_angles)
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

        targets_in_radians = [radians(angle) for angle in self.saved_positions[0]["joint_angles"]]
        self.sync_write_angles(targets_in_radians)

        time.sleep(1)   # This delay is necessary for the MCU to finish processing the previous command.

        self.update_position_by_alias("HOME")

        time.sleep(delay)

        new_angles = [round(degrees(joint["angle"]), 1) for joint in self.joint_info.values()]

        return new_angles