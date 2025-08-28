"""
Robot class for simulation-specific controls (no physical motors).
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
from robot_arm.robot_core import RobotCore

# Initialize logger
logger = logging.getLogger(__name__)
logging.basicConfig(filename="api_logs.log", level=logging.INFO)

class SimulationRobot:
    """Class for simulation environment."""

    def __init__(self):
        self._k = Kinematics("config.json")
        self._core = RobotCore()

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

        self.speed = config["servo_params"]["default_speed"]
        self.accel = config["servo_params"]["default_accel"]
        self.MOVE_DELAY = 0

        # Initialize joint info dictionary
        self._core.joint_info = {
            1: {
                "angle": 0,
                "speed": self.speed,
                "accel": self.accel,
                "min_angle": radians(config["joint_limits"]["joint_1"]["min_angle"]),
                "max_angle": radians(config["joint_limits"]["joint_1"]["max_angle"]),
                "enabled": 0
            },
            2: {
                "angle": 0,
                "speed": self.speed,
                "accel": self.accel,
                "min_angle": radians(config["joint_limits"]["joint_2"]["min_angle"]),
                "max_angle": radians(config["joint_limits"]["joint_2"]["max_angle"]),                
                "enabled": 0
            },
            3: {
                "angle": 0,
                "speed": self.speed,
                "accel": self.accel,
                "min_angle": radians(config["joint_limits"]["joint_3"]["min_angle"]),
                "max_angle": radians(config["joint_limits"]["joint_3"]["max_angle"]),                
                "enabled": 0
            },
            4: {
                "angle": 0,
                "speed": self.speed+250,
                "accel": self.accel,
                "min_angle": radians(config["joint_limits"]["joint_4"]["min_angle"]),
                "max_angle": radians(config["joint_limits"]["joint_4"]["max_angle"]),                
                "enabled": 0
            },
            5: {
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
        # self._core.saved_positions = {
        #     0: {
        #         "alias": "HOME",
        #         "joint_angles": config["defined_positions"]["home"],     # A list of angles in degrees
        #         "ee_position": [],
        #         "ee_orientation": []
        #     }
        # }

        # Close file when finished
        f.close()

    def compute_fk(self) -> np.ndarray:
        """Calculate the transformation matrix of the current end effector pose.
        
        Returns
        -------
        A numpy array for the 4x4 matrix that represents the rotation and position of the end effector.
        """
        
        # TODO: Call a read joint angle method here?

        joint_angles = [joint["angle"] for joint in self._core.joint_info.values()]   # List comprehension to extract angle values from dictionary

        FK = self._k.calc_fk_mat(joint_angles)

        return FK

    def get_ee_pos(self) -> np.ndarray:
        """ Get the end effector's position in Cartesian coordinates relative to the base frame.
        
        Returns
        -------
        A numpy array for the 3x1 position vector.
        """

        fk_mat = self.compute_fk()
        pos_vec = fk_mat[:3, 3].transpose()

        return pos_vec
    
    def get_ee_rot(self) -> np.ndarray:
        """Get the end effector's rotation matrix relative to the base frame.
        
        Returns
        -------
        A numpy array for the 3x3 rotation matrix.
        """

        fk_mat = self.compute_fk()
        rot_mat = fk_mat[:3,:3]

        return rot_mat
    
    def compute_ik_from_pos(self, pos_vec: list[float]) -> list[float]:
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
            print(f"Error calculating transformation matrix during IK computation: {e}")
        
        target_joint_angles = self._k.calc_all_joint_angles(target_frame)

        return target_joint_angles
    
    def get_joint_angles_from_tf(self) -> list[float]:
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

    def internal_update_joint_angles(self, joint_angles: list[float]):
        """
        Updates internal saved joint angles in DEGREES.

        Takes a list of floats in RADIANS.
        """

        for joint_counter, joint in enumerate(self.joint_info.values()):
            joint["angle"] = round(degrees(joint_angles[joint_counter]), 2)
    
    def move_x(self, x: float, delay=None) -> list[float]:
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
        self.internal_update_joint_angles(target_joint_angles)
        time.sleep(delay)

        new_angles = [round(degrees(joint["angle"]), 1) for joint in self.joint_info.values()]

        return new_angles
    
    def move_y(self, y: float, delay=None) -> list[float]:
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
        self.internal_update_joint_angles(target_joint_angles)
        time.sleep(delay)

        new_angles = [round(degrees(joint["angle"]), 1) for joint in self.joint_info.values()]


        return new_angles
    
    def move_z(self, z: float, delay=None) -> list[float]:
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
        self.internal_update_joint_angles(target_joint_angles)
        time.sleep(delay)

        new_angles = [round(degrees(joint["angle"]), 1) for joint in self.joint_info.values()]

        return new_angles

    def home(self, delay=None) -> list[float]:
        """Go to the home position defined in config.json.
        
        This method sets a manual target angle to each servo, so the arm must be calibrated beforehand.

        Returns a list of updated joint angles in degrees.

        TODO: Properly implement the delay in the library.
        """

        if delay==None:
            delay = self._core.MOVE_DELAY

        targets_in_radians = [radians(angle) for angle in self._core.saved_positions[0]["joint_angles"]]
        new_angles = [round(degrees(angle), 1) for angle in targets_in_radians]

        return new_angles