""" A kinematics library for a 5-DOF robot arm.

The calculations are specific to an RRRRR articulated robot in a particular configuration.
"""

from math import sin, cos, atan2, pi, sqrt, acos, asin
import json

import numpy as np


class Kinematics():
    """
    A class to encompass the kinematics calculations for a 5-DOF articulated robot arm.

    Attributes
    ----------
    DH_PARAMS: dict
        A dictionary containing the robot's Denavit-Hartenberg parameters.

    Methods
    -------
    calc_fk_mat: Compute the transformation matrix of the end effector using known joint angles.

    calc_all_joint_angles: Use inverse kinematics to compute all joint angles from a given transformation matrix.

    calc_tf_from_position: Use geometric transformation to get a transformation matrix for a translation in Cartesian space.

    calc_tf_from roll: Calculate the transformation matrix to roll the wrist.

    calc_tf_from_pitch: Calculate the transformation matrix to pitch the wrist.

    calc_tf_from yaw: Calculate the transformation matrix to yaw the wrist.
    """

    def __init__(self, config_file: str):
        self._load_DH_params(config_file)     # Load DH parameters from config file

    def _load_DH_params(self, file_name: str):
        """Load DH parameters from a config file (JSON format) and stores them in a dictionary as a class attribute.
        
        Args
        ----
        file_name: str
            The file name of the config file (including .json extension).

        Return
        ------
        None
        """

        try:
            with open(file_name, "r") as f:
                config = json.load(f)
        except:
            print(f"{file_name} not found.")
            exit()

        self.DH_PARAMS = {
            "joint_1": {
                "d": config["DH_params"]["joint_1"]["d"],
                "theta_offset": config["DH_params"]["joint_1"]["theta_offset"],
                "a": config["DH_params"]["joint_1"]["a"],
                "alpha": config["DH_params"]["joint_1"]["alpha"]
            },
            "joint_2": {
                "d": config["DH_params"]["joint_2"]["d"],
                "theta_offset": config["DH_params"]["joint_2"]["theta_offset"],
                "a": config["DH_params"]["joint_2"]["a"],
                "alpha": config["DH_params"]["joint_2"]["alpha"]
            },
            "joint_3": {
                "d": config["DH_params"]["joint_3"]["d"],
                "theta_offset": config["DH_params"]["joint_3"]["theta_offset"],
                "a": config["DH_params"]["joint_3"]["a"],
                "alpha": config["DH_params"]["joint_3"]["alpha"]
            },
            "joint_4": {
                "d": config["DH_params"]["joint_4"]["d"],
                "theta_offset": config["DH_params"]["joint_4"]["theta_offset"],
                "a": config["DH_params"]["joint_4"]["a"],
                "alpha": config["DH_params"]["joint_4"]["alpha"]
            },
            "joint_5":  {
                "d": config["DH_params"]["joint_5"]["d"],
                "theta_offset": config["DH_params"]["joint_5"]["theta_offset"],
                "a": config["DH_params"]["joint_5"]["a"],
                "alpha": config["DH_params"]["joint_5"]["alpha"]
            }
        }
        
        f.close()

    # =======================
    # FORWARD KINEMATICS
    # =======================

    def get_frame_mat(self, nx: float, ny: float, nz: float, ox: float, oy: float, oz: float, ax: float, ay: float, az: float, Px: float, Py: float, Pz: float):
        """Build a 4x4 frame matrix.

        Args
        -----
        Yes

        Returns
        -------
        NDArray
            A numpy array for the frame matrix.
        """

        frame_matrix = np.array([[nx, ox, ax, Px],
                                [ny, oy, ay, Py],
                                [nz, oz, az, Pz],
                                [0, 0, 0, 1]])
        
        return frame_matrix

    def _get_tf_mat(self, d: float, theta: float, a: float, alpha: float):
        """Build the 4x4 transformation matrix based on the DH parameters.
        
        Args
        ----
        d: float

        theta: float

        a: float

        alpha: float

        Returns
        -------
        NDArray
            A numpy array for the resultant transformation matrix.

        """

        tf_mat = np.array([[cos(theta), -(sin(theta))*(cos(alpha)), (sin(theta))*(sin(alpha)), a*cos(theta)],
                            [sin(theta), (cos(theta))*(cos(alpha)), -(cos(theta))*(sin(alpha)), a*sin(theta)],
                            [0,           sin(alpha),                 cos(alpha),                 d],
                            [0,           0,                          0,                          1]])
        
        return tf_mat

    def calc_fk_mat(self, joint_angles:list):
        """Calculate the forward kinematics transformation matrix.

        Forward kinematics takes the robot's current joint angles (in radians) and calculates the end effector's position
        and orientation in Cartesian space based on the DH parameters. The transformation matrix of each joint is computed,
        then they are post-multiplied in sequence to generate the final homogeneous transformation matrix.

        Args
        ----
        joint_angles: list
            A list of the robot's current joint angles in RADIANS.

        Returns
        -------
        NDArray
            The 4x4 homogeneous transformation matrix that describes the end effector's current pose.
        """

        joint_T_mat_list = []

        for idx, joint in enumerate(self.DH_PARAMS.keys()):
            joint_T_mat = self._get_tf_mat(self.DH_PARAMS[joint]["d"],
                                            joint_angles[idx] + self.DH_PARAMS[joint]["theta_offset"],
                                            self.DH_PARAMS[joint]["a"],
                                            self.DH_PARAMS[joint]["alpha"]
                                        )
            joint_T_mat_list.append(joint_T_mat)

        T_mat = joint_T_mat_list[0] @ joint_T_mat_list[1] @ joint_T_mat_list[2] @ joint_T_mat_list[3] @ joint_T_mat_list[4]

        return T_mat
    
    # ===================
    # INVERSE KINEMATICS
    # ===================

    def _rot_mat_12(theta_1: float):
        """Calculate the rotation matrix from frame 1 to frame 2.
        
        TODO: See if this method is ever useful.
        """
        pass

    def _rot_mat_46(self, theta_4: float, theta_5: float):
        """The rotation matrix from frame 4 to frame 6.
        
        Args
        ----
        theta_4: float
            The angle of joint 4 in radians.

        theta_5: float
            The angle of joint 5 in radians.

        Returns
        -------
        NDArray
            A numpy array for the 3x3 rotation matrix from frame 4 to frame 6.
        """

        r_11 = -sin(theta_4) * cos(theta_5)
        r_21 = cos(theta_4) * cos(theta_5)
        r_31 = sin(theta_5)
        r_12 = sin(theta_4) * sin(theta_5)
        r_22 = -cos(theta_4) * sin(theta_5)
        r_32 = cos(theta_5)
        r_13 = cos(theta_4)
        r_23 = sin(theta_4)

        rot_mat = np.array([[r_11, r_12, r_13],
                            [r_21, r_22, r_23],
                            [r_31, r_32, 0]])
        
        return rot_mat

    def _rot_mat_14(self, theta_1, theta_2, theta_3):
        """The rotation matrix from frame 1 to frame 4.
        
        Args
        ----
        theta_1: float
            The angle of joint 1 in radians.

        theta_2: float
            The angle of joint 2 in radians.

        theta_3: float
            The angle of joint 3 in radians.

        Returns
        -------
        NDArray
            A numpy array for the 3x3 rotation matrix from frame 1 to frame 4.
        """

        r_11 = -cos(theta_1) * sin(theta_2) * cos(theta_3) - cos(theta_1) * cos(theta_2) * sin(theta_3)
        r_21 = -sin(theta_1) * sin(theta_2) * cos(theta_3) - sin(theta_1) * cos(theta_2) * sin(theta_3)
        r_31 = cos(theta_2) * cos(theta_3) - sin(theta_2) * sin(theta_3)
        r_12 = cos(theta_1) * sin(theta_2) * sin(theta_3) - cos(theta_1) * cos(theta_2) * cos(theta_3)
        r_22 = sin(theta_1) * sin(theta_2) * sin(theta_3) - sin(theta_1) * cos(theta_2) * cos(theta_3)
        r_32 = -cos(theta_2) * sin(theta_3) - sin(theta_2) * cos(theta_3)
        r_13 = sin(theta_1)
        r_23 = -cos(theta_1)

        rot_mat = np.array([[r_11, r_12, r_13],
                            [r_21, r_22, r_23],
                            [r_31, r_32, 0]])
        
        return rot_mat


    def _rot_mat_16(tf_matrix):
        # ???
        pass


    # ==================
    # INVERSE KINEMATICS
    # ==================

    def _calc_wrist_position(self, tf_mat):
        """Calculate the position of the wrist in Cartesian coordinates.

        Use inverse kinematics to compute the position of the wrist where the global origin is defined in the kinematic diagram.

        Args
        ----
        tf_mat: NDArray
            A 4x4 homogeneous transformation matrix for the end effector pose as a numpy array.

        Returns
        -------
        wrist_pos_vec: NDArray
            A 3x1 vector that represents the xyz coordinates of the wrist.
        """

        rot_mat = tf_mat[:3,:3]
        pos_vec = tf_mat[:3, 3].transpose()

        wx = pos_vec[0] - (self.DH_PARAMS["joint_5"]["d"] * rot_mat[:3, 2][0].transpose())
        wy = pos_vec[1] - (self.DH_PARAMS["joint_5"]["d"] * rot_mat[:3, 2][1].transpose())
        wz = pos_vec[2] - (self.DH_PARAMS["joint_5"]["d"] * rot_mat[:3, 2][2].transpose())

        wrist_pos_vec = np.array([wx, wy, wz])
        
        return wrist_pos_vec

    def _calc_wrist_orientation(self, tf_mat):
        """Calculate the orientation of the wrist in Cartesian space.

        Use inverse kinematics to compute the orientation of the wrist where the global origin is defined in the kinematic diagram.

        Args
        ----
        tf_mat: NDArray
            A 4x4 homogeneous transformation matrix for the end effector pose as a numpy array.

        Returns
        -------
        rot_vecs: list[NDArray, ...]
            A list of 3x1 vectors that represents the rotations about the x, y, and z axes of the wrist.
        """

        rot_mat = tf_mat[:3,:3]

        rot_x = rot_mat[:3, 0].transpose()
        rot_y = rot_mat[:3, 1].transpose()
        rot_z = rot_mat[:3, 2].transpose()

        rot_vecs = [rot_x, rot_y, rot_z]

        return rot_vecs

    # This function is a faster way of doing the FK for the wrist for rotation movements
    def _calc_wrist_pose(self, tf_mat):
        """Compute the transformation matrix that describes the wrist pose.

        Use inverse kinematics to compute the pose of the wrist from the global origin, as defined in the kinematic diagram.

        Args
        ----
        tf_mat: NDArray
            A 4x4 homogeneous transformation matrix for the end effector pose as a numpy array.

        Returns
        -------
        wrist_tf: NDArray
            A numpy array for the 4x4 transformation matrix that represents the wrist pose.
        """

        rot_mat = tf_mat[:3,:3]
        pos_vec = tf_mat[:3, 3].transpose()

        tf_mat[0][3] = pos_vec[0] - (self.DH_PARAMS["joint_5"]["d"] * rot_mat[:3, 2][0].transpose())
        tf_mat[1][3] = pos_vec[1] - (self.DH_PARAMS["joint_5"]["d"] * rot_mat[:3, 2][1].transpose())
        tf_mat[2][3] = pos_vec[2] - (self.DH_PARAMS["joint_5"]["d"] * rot_mat[:3, 2][2].transpose())

        wrist_tf = tf_mat
        
        return wrist_tf


    def calc_ee_pose(self, tf_mat):
        """
        Wait is this just a roundabout way to return the tf_mat? lol

        TODO: Test with some print statements to see if the output tf_mat is the same as the input ee_tf
        """

        rot_mat = tf_mat[:3,:3]
        pos_vec = tf_mat[:3, 3].transpose()

        tf_mat[0, 3] = pos_vec[0] + (self.DH_PARAMS["joint_5"]["d"] * rot_mat[:3, 2][0].transpose())
        tf_mat[1, 3] = pos_vec[1] + (self.DH_PARAMS["joint_5"]["d"] * rot_mat[:3, 2][1].transpose())
        tf_mat[2, 3] = pos_vec[2] + (self.DH_PARAMS["joint_5"]["d"] * rot_mat[:3, 2][2].transpose())

        ee_tf = tf_mat
        
        return ee_tf


    def _calc_theta_1(self, wrist_pos_vector):
        """Calculate the first joint variable defined in the robot's kinematic diagram.

        Given the wrist position vector, compute the robot's first joint variable using the atan2() function.

        Args
        ----
        wrist_pos_vector: list
            The wrist position vector as a list of Cartesian coordinates.

        Returns
        -------
        A float of the first joint variable theta_1 in radians. This is the angle on the x-y plane between the wrist and the base joint.
        
        """
        wy = wrist_pos_vector[1]
        wx = wrist_pos_vector[0]

        try:
            theta_1 = atan2(wy, wx)
        except:
            print(f"""
                Invalid math operation when trying to compute theta_1 using atan2() with the following parameters:
                wy: {wy}
                wx: {wx}
            """)

        return theta_1


    def _calc_theta_3(self, wrist_pos_vector):
        """Calculate the third joint variable defined in the robot's kinematic diagram.
        
        Given the wrist position vector, compute the robot's third joint variable using the cosine law. This value is required to calculate theta_2.

        Args
        ----
        wrist_pos_vector: list
            The wrist position vector as a list of Cartesian coordinates.

        Returns
        -------
        A float of the third joint variable theta_3 in radians. This angle is measured from the axis of the link arm connecting joint 2 and joint 3.
        """
        wy = wrist_pos_vector[1]
        wx = wrist_pos_vector[0]
        wz = wrist_pos_vector[2]

        a = sqrt((wx**2) + (wy**2))
        b = wz - self.DH_PARAMS["joint_1"]["d"]  # wz - d1

        numerator = (a**2) + (b**2) - (self.DH_PARAMS["joint_2"]["a"] ** 2) - (self.DH_PARAMS["joint_3"]["a"] ** 2)
        denominator = 2 * self.DH_PARAMS["joint_2"]["a"] * self.DH_PARAMS["joint_3"]["a"]

        try:
            theta_3 = acos(numerator/denominator)
        except:
            print(f"""
                Invalid math operation when trying to compute theta_3 using arccos() with the following parameters:
                numerator: {numerator}
                denominator: {denominator}
            """)

        return theta_3


    def _calc_theta_2(self, wrist_pos_vector, theta_3):
        """Calculate the second joint variable defined in the robot's kinematic diagram.
        
        Given the wrist position vector and theta_3, compute the robot's second joint variable using the cosine law.

        Args
        ----
        wrist_pos_vector: list
            The wrist position vector as a list of Cartesian coordinates.

        theta_3: float
            The third joint variable in radians.

        Returns
        -------
        A float of the second joint variable in radians. This angle is measured from the axis of the link arm connecting joint 1 and joint 2.
        """

        wy = wrist_pos_vector[1]
        wx = wrist_pos_vector[0]
        wz = wrist_pos_vector[2]

        a = sqrt((wx**2) + (wy**2))
        b = wz - self.DH_PARAMS["joint_1"]["d"]  # wz - d1

        numerator = ((self.DH_PARAMS["joint_2"]["a"] + self.DH_PARAMS["joint_3"]["a"] * cos(theta_3)) * a) + (b * self.DH_PARAMS["joint_3"]["a"] * sin(theta_3))
        denominator = (a**2) + (b**2)

        try:
            theta_2 = acos(numerator/denominator)
        except:
            print(f"""
                Invalid math operation when trying to compute theta_2 using arccos() with the following parameters:
                numerator: {numerator}
                denominator: {denominator}
            """)

        return theta_2


    def _calc_wrist_position_angles(self, tf_matrix):
        """Calculate the joint angles that dictate the wrist position.

        The first 3 joint variables determine the position of the wrist origin.

        Args
        ----
        tf_matrix: NDArray
            A numpy array for the homogeneous tranformation matrix that defines the end effector's current pose.

        Returns
        -------
        A list of angles (in radians) for the first 3 joint variables, ordered from 1-3.
        
        """

        wrist_pos = self._calc_wrist_position(tf_matrix)
        theta_1 = round(self._calc_theta_1(wrist_pos), 5)
        theta_3 = round(self._calc_theta_3(wrist_pos), 5)
        
        theta_2 = pi/2 - round(self._calc_theta_2(wrist_pos, -theta_3), 5)     # Subtract from pi/2 because of frame assignment, flip the sign on theta_3 because of the starting reference of the angle measurement

        wrist_pos_angles = [theta_1, theta_2, theta_3]

        return wrist_pos_angles


    def calc_all_joint_angles(self, tf_matrix):
        """Calculate all target joint angles using inverse kinematics.

        Args
        ----
        tf_matrix: NDArray
            A numpy array for the homogeneous tranformation matrix that defines the end effector's current pose.

        Returns
        -------
        A list of angles (in radians) for all 5 joint variables, ordered from 1-5.                
        """

        joint_angles = self._calc_wrist_position_angles(tf_matrix)
        rot_mat_16 = tf_matrix[:3, :3]

        rot_matrix_14 = self._rot_mat_14(joint_angles[0], joint_angles[1], joint_angles[2])
        rot_mat_14_inv = rot_matrix_14.transpose()

        rot_mat_product = rot_mat_14_inv @ (rot_mat_16)

        theta_4 = round(asin(rot_mat_product[1, 2]), 2)    # Use sin to avoid some solution ambiguity (cos is symmetric about 0)
        theta_5 = round(asin(rot_mat_product[2, 0]), 2)    # Use sin to avoid some solution ambiguity

        joint_angles.append(theta_4)
        joint_angles.append(theta_5)

        return joint_angles


    def _get_wrist_orientation_angles(self, joint_angles:list):
        """Slice the list of joint angles to return only the wrist orientation angles (last 2 values).
        
        Args
        ----
        joint_angles: list
            A list of all the joint angles in radians, ordered from 1-5.

        Returns
        -------
        A list of the joint angles that describe the wrist's orientation (theta_4 and theta_5).
        """

        return joint_angles[3:]


    """
    TRANSLATIONS AND ROTATION
    """

    def calc_tf_from_position(self, position_vec: list, current_frame):
        """Calculate the transformation matrix to move the end effector to a new position while keeping the same orientation.

        This method computes the transformation matrix for relative motion from the current point to a new point in Cartesian space using geometric transformations.
        
        Args
        ----
        position_vec: list
            A 3x1 position vector as a list of Cartesian coordinates describing the target point relative to the current end effector position.
        
        current_frame: NDArray
            A 4x4 numpy array describing the current end effector pose.
        
        Returns
        -------
        A numpy array for the homogeneous transformation matrix that describes the absolute target pose in Cartesian space.
        """
        print("calculating tf from position")
        translation_mtx = np.array([[1, 0, 0, position_vec[0]],
                                    [0, 1, 0, position_vec[1]],
                                    [0, 0, 1, position_vec[2]],
                                    [0, 0, 0, 1]])

        new_tf_mat = translation_mtx @ current_frame

        return new_tf_mat

    # ===================
    # WRIST MANIPULATIONS
    # ===================

    """
    roll = rotation about x
    pitch = rotation about y
    yaw = rotation about z    
    """

    def calc_tf_from_roll(self, roll_angle: float, current_frame):
        """Calculate the transformation matrix to roll the wrist.

        Roll is defined as a rotation about the wrist's x-axis.
        
        Args
        ----
        roll_angle: float
            The angle to roll in radians.
        
        current_frame: NDArray
            A numpy array for the 4x4 transformation matrix that describes the end effector's current pose.

        Returns
        -------
            A numpy array for the 4x4 transformation matrix that describes the end effector's new pose after rolling.
        """

        rotation_mtx = np.array([[1, 0, 0, 0],
                                [0, cos(roll_angle), -sin(roll_angle), 0],
                                [0, sin(roll_angle), cos(roll_angle), 0],
                                [0, 0, 0, 1]])

        new_frame = rotation_mtx @ current_frame

        return new_frame


    def calc_tf_from_pitch(self, pitch_angle: float, current_frame):
        """Calculate the transformation matrix to pitch the wrist.

        Pitch is defined as a rotation about the wrist's y-axis.
        
        Args
        ----
        pitch_angle: float
            The angle to pitch in radians.
        
        current_frame: NDArray
            A numpy array for the 4x4 transformation matrix that describes the end effector's current pose.

        Returns
        -------
            A numpy array for the 4x4 transformation matrix that describes the end effector's new pose after pitching.
        """        

        rotation_mtx = np.array([[cos(pitch_angle), 0, sin(pitch_angle), 0],
                                [0, 1, 0, 0],
                                [-sin(pitch_angle), 0, cos(pitch_angle), 0],
                                [0, 0, 0, 1]])
        
        new_frame = current_frame @ rotation_mtx

        return new_frame


    def calc_tf_from_yaw(self, yaw_angle: float, current_frame):
        """Calculate the transformation matrix to yaw the wrist.

        Yaw is defined as a rotation about the wrist's z-axis.
        
        Args
        ----
        yaw_angle: float
            The angle to yaw in radians.
        
        current_frame: NDArray
            A numpy array for the 4x4 transformation matrix that describes the end effector's current pose.

        Returns
        -------
            A numpy array for the 4x4 transformation matrix that describes the end effector's new pose after yawing.
        """        
        rotation_mtx = np.array([[cos(yaw_angle), -sin(yaw_angle), 0, 0],
                                [sin(yaw_angle), cos(yaw_angle), 0, 0],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])
        
        new_frame = current_frame @ rotation_mtx

        return new_frame