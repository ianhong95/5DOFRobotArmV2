"""
Overarching class to coordinate the physical robot and simulated robot.

The purpose of this class is to call high level methods that invoke the lower level
implementation of the robot methods of whichever robot is active (physical, simulation,
or both).
"""

import json
import logging
import time
from math import degrees, radians

import numpy as np

from robot_arm.physical_robot import PhysicalRobot
from robot_arm.simulation_robot import SimulationRobot

class RobotManager:
    """Robot environment coordinator."""

    def __init__(self, physical_robot_connected = 0, simulation_robot_connected = 0):
        self.physical_robot_connected = physical_robot_connected
        self.simulation_robot_connected =  simulation_robot_connected

        self.physical_robot = None
        self.simulation_robot = None

        self._load_robots()

    def _load_robots(self):
        """Selectively load robot environment(s)."""
        if self.physical_robot_connected:
            self.physical_robot = PhysicalRobot()
        else:
            print("No physical robot.")
        
        if self.simulation_robot_connected:
            self.simulation_robot = SimulationRobot()
        else:
            print("No simulation robot.")

    def home(self) -> list[float]:
        """
        Command the robot to move to the home position.
        
        Returns a list of angles in degrees.
        """
        
        if self.physical_robot:
            angles_in_degrees = self.physical_robot.home()
            xyz_position = (self.physical_robot.get_ee_pos()).tolist()
        
        if self.simulation_robot:
            angles_in_degrees = self.simulation_robot.home()
            xyz_position = (self.simulation_robot.get_ee_pos()).tolist()

        return xyz_position, angles_in_degrees
    
    def disable(self):
        """Disables the physical robot. No implementation for simulation robot."""
        if self.physical_robot:
            self.physical_robot.disable_servo(0)

    def read_joint_angles(self) -> list[float]:
        """
        Reads the current joint angles of the robot.
        
        Returns a list of angles in degrees.
        """

        if self.physical_robot:
            angles_in_radians = self.physical_robot.read_joint_angle(0)
        else:
            # angles_in_radians = self.simulation_robot.read_joint_angles()
            pass

        angles_in_degrees = [round(degrees(angle), 2) for angle in angles_in_radians]

        return angles_in_degrees

    def get_ee_pos(self) -> np.ndarray[float]:
        """
        Reads the current end effector position in Cartesian coordinates.
        
        Returns a 3x1 numpy array of floats.
        """

        if self.physical_robot:
            xyz_position = (self.physical_robot.get_ee_pos()).tolist()
        else:
            xyz_position = (self.simulation_robot.get_ee_pos()).tolist()
        
        rounded_xyz_position = [round(coordinate, 2) for coordinate in xyz_position]

        return rounded_xyz_position

    def move_x(self, x_distance: float):
        """Moves along the x axis by a specified distance from th current positions."""

        if self.physical_robot:
            self.physical_robot.move_x(x_distance)
        
        self.simulation_robot.move_x(x_distance)

    def move_y(self, y_distance: float):
        """Moves along the y axis by a specified distance from th current positions."""

        if self.physical_robot:
            self.physical_robot.move_y(y_distance)
        
        self.simulation_robot.move_y(y_distance)

    def move_z(self, z_distance: float):
        """Moves along the z axis by a specified distance from th current positions."""

        if self.physical_robot:
            self.physical_robot.move_z(z_distance)
        
        self.simulation_robot.move_z(z_distance)

    def set_gripper(self, state: bool):
        """
        Open/close the gripper.
        
        Pass a boolean as an argument. True means open gripper, false means close gripper.
        """

        if self.physical_robot:
            self.physical_robot.set_gripper_state(state)
        
        # self.simulation_robot.set_gripper_state(state)
    
    def save_current_position(self) -> list[float]:
        """Save the current joint positions to the database."""

        if self.physical_robot:
            joint_angles_radians = self.physical_robot.read_joint_angle(0)
            xyz_position = (self.robot_arm.get_ee_pos()).tolist()
        else:
            # joint_angles_radians = self.simulation_robot.read_joint_angle(0)
            xyz_position = (self.simulation_robot.get_ee_pos()).tolist()

        joint_angles_degrees = [round(degrees(angle), 2) for angle in joint_angles_radians]
        rounded_xyz_position = [round(coord, 2) for coord in xyz_position]

        return rounded_xyz_position, joint_angles_degrees
    
    def write_joint_angles(self, joint_angles: list[float]):
        """Write target angles to each joint."""

        if self.physical_robot:
            self.physical_robot.sync_write_angles(joint_angles)

        # Implement a method to write angles to simulation robot