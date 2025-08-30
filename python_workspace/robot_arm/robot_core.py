"""
This is a library for the RobotArm class that contains all the attributes and methods to control the arm.

TODO:
- Add more debugging methods
- Clean up hacky code
- Continuously read position?
- Implement deadband?
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
from robot_arm.kinematics import Kinematics

# Initialize logger
logger = logging.getLogger(__name__)
logging.basicConfig(filename="api_logs.log", level=logging.INFO)


class RobotCore:
    """Robot arm class for scripting motions.

    Attributes
    ----------
    joint_info: Dictionary of joint properties such as ID, angle, speed, etc.

    current_tf: Numpy array of the transformation matrix for the end effector's current pose.
    
    Methods
    -------
    home(): Move the robot to the "home" position defined in the config.json.

    move_x(x): Move a distance x along the global x axis relative to the current position.

    move_y(y): Move a distance y along the global y axis relative to the current position.

    move_z(z): Move a distance z along the global z axis relative to the current position.
    """

    def __init__(self, sim: int = 0):
        # Load kinematics library class
        self._k = Kinematics("config.json")

        # Set up connection to robot
        self._load_config()
            
        self.current_tf = self.compute_fk() # Initialize kinematics

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

        self.GRIPPER_ID = config["gripper_params"]["id"]
        self.GRIPPER_CLOSE_POS = config["gripper_params"]["close_pos"]
        self.GRIPPER_OPEN_POS = config["gripper_params"]["open_pos"]
        self.GRIPPER_TOLERANCE = config["gripper_params"]["tolerance"]

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

    # ==================
    # VALIDATION METHODS
    # ==================

    def _check_joint_limits(self, joint_idx: int, target_angle: float):
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

    def set_speed(self, speed:int):
        """Globally sets the speed. I don't think this is very useful right now."""

    # ===========
    # KINEMATICS
    # ===========

    def compute_fk(self):
        """Calculate the transformation matrix of the current end effector pose.
        
        Returns
        -------
        A numpy array for the 4x4 matrix that represents the rotation and position of the end effector.
        """
        
        # TODO: Call a read joint angle method here?

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
        joint_angles = self._k.calc_all_joint_angles(fk)

        deg_angles = [angle for angle in joint_angles]

        return deg_angles