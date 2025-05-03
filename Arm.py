import numpy as np
from Simulation import Simulation
import matplotlib.pyplot as plt
import math

class Arm:
    # Constructor
    def __init__(self, start_position):
        self.start_position = start_position # tuple
        # Instead have a function that can add lengths and linkeages to an array
        self.linkeage_angles = [] # angle of linkeages relative to the previous linkeage (in radians)
        self.linkeage_lengths = [] # in meters or change to like pixels? (floats)

    # Adds a new linkeage of a given angle and length to the arrays 
    def add_linkeage(self, angle, length):
        self.linkeage_angles.append(angle)
        self.linkeage_lengths.append(length)

    # removes the last linkeage
    def remove_linkeage(self):
        if self.linkeage_angles and self.linkeage_lengths:
            self.linkeage_angles.pop()
            self.linkeage_lengths.pop()

    # Based on the linkeages' length and angle, return an x, y of the final/end effector's position
    def calculate_end_position(self):
        current_position = self.start_position
        total_angle = 0

        for i in range(len(self.linkeage_angles)):
            total_angle += self.linkeage_angles[i]
            length = self.linkeage_lengths[i]

            current_position = (
                current_position[0] + length * np.cos(total_angle),
                current_position[1] + length * np.sin(total_angle)
            )

        return current_position


    # given an array of angles, change the arm's angles to the new angles
    # renormlizes angles to be between -pi and pi
    def change_angles(self, array_of_angles):
        self.linkeage_angles = array_of_angles

        self.linkeage_angles = [(angle + np.pi) % (2 * np.pi) - np.pi for angle in array_of_angles]


    # given a matplot plot, draw yourself on said plot
    def draw(self, plt):
        current_position = self.start_position
        total_angle = 0

        for i in range(len(self.linkeage_angles)):
            total_angle += self.linkeage_angles[i]
            length = self.linkeage_lengths[i]

            next_point = (
                current_position[0] + length * np.cos(total_angle),
                current_position[1] + length * np.sin(total_angle)
            )

            x_vals = [current_position[0], next_point[0]]
            y_vals = [current_position[1], next_point[1]]

            plt.plot(x_vals, y_vals)
            current_position = next_point

    
    # Returns a list of (x, y) positions for all joints, starting at the base and ending at the end effector.
    # Used in animation in notebook
    def get_joint_positions(self):
        positions = [self.start_position]
        total_angle = 0

        for i in range(len(self.linkeage_angles)):
            prev = positions[-1]
            total_angle += self.linkeage_angles[i]
            length = self.linkeage_lengths[i]

            next_pos = (
                prev[0] + length * np.cos(total_angle),
                prev[1] + length * np.sin(total_angle)
            )
            positions.append(next_pos)

        return positions


    
    # calculate the Jacobian matrix for the arm
    # The Jacobian matrix relates the change in joint angles to the change in end effector position
    def calculate_jacobian(self):
        n = len(self.linkeage_angles)
        J = np.zeros((2, n))
        joint_positions = self.get_joint_positions()
        end_effector = np.array(joint_positions[-1])

        for i in range(n):
            joint_pos = np.array(joint_positions[i])
            r = end_effector - joint_pos  # vector from joint to end effector

            # 2D perpendicular vector (∂x/∂θ = -r_y, ∂y/∂θ = r_x)
            J[0, i] = -r[1]
            J[1, i] =  r[0]

        return J

        
