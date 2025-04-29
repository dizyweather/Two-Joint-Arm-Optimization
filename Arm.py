import numpy as np
from Simulation import Simulation
import matplotlib.pyplot as plt
import math

class Arm:
    # Constructor
    # Could have an empty constrcutor
    def __init__(self, start_position):
        self.start_position = start_position # tuple
        # Instead have a function that can add lengths and linkeages to an array
        self.linkeage_angles = [] # angle of lineage in radians in global frame (floats)
        self.linkeage_lengths = [] # in meters or change to like pixels? (floats)

    # Adds a new linkeage to the arrays
    def add_linkeage(self, angle, length):
        self.linkeage_angles.append(angle)
        self.linkeage_lengths.append(length)

    def remove_linkeage(self):
        if self.linkeage_angles and self.linkeage_lengths:
            self.linkeage_angles.pop()
            self.linkeage_lengths.pop()

    # Based on the linkeages' length and angle, return an x, y of the final/end effector's position
    def calculate_end_position(self):
        current_position = self.start_position

        for i in range(len(self.linkeage_angles)):
            current_length = self.linkeage_lengths[i]
            current_angle = self.linkeage_angles[i]

            current_position = (current_position[0] + current_length * np.cos(current_angle), 
                          current_position[1] + current_length * np.sin(current_angle))

        return current_position

    def change_angles(self, array_of_angles):
        self.linkeage_angles = array_of_angles

        for i in range(len(self.linkeage_angles)):
            if self.linkeage_angles[i] > np.pi:
                self.linkeage_angles[i] -= 2 * np.pi
            elif self.linkeage_angles[i] < -np.pi:
                self.linkeage_angles[i] += 2 * np.pi

    # given a matplot plot, draw yourself on said plot
    def draw(self, plt):
        plt.clf() # clear figure

        current_position = self.start_position

        for i in range(len(self.linkeage_angles)):
            current_length = self.linkeage_lengths[i]
            current_angle = self.linkeage_angles[i]

            next_point = (current_position[0] + current_length * np.cos(current_angle), 
                          current_position[1] + current_length * np.sin(current_angle))
            
            x_vals = [current_position[0], next_point[0]]
            y_vals = [current_position[1], next_point[1]]

            plt.plot(x_vals, y_vals)

            current_position = next_point
    
    # Returns a list of (x, y) positions for all joints, starting at the base and ending at the end effector.
    def get_joint_positions(self):

        positions = [self.start_position]
        

        for i in range(len(self.linkeage_angles)):
            previous_position = positions[i]
            length = self.linkeage_lengths[i]
            angle = self.linkeage_angles[i]

            x = previous_position[0] + length * math.cos(angle)
            y = previous_position[1] + length * math.sin(angle)
            positions.append((x, y))

        return positions

    
    # calculate the Jacobian matrix for the arm
    # The Jacobian matrix relates the change in joint angles to the change in end effector position
    def calculate_jacobian(self):
        n = len(self.linkeage_angles)
        J = np.zeros((2, n))

        current_position = self.start_position
        for i in range(n):
            current_length = self.linkeage_lengths[i]
            current_angle = self.linkeage_angles[i]

            # Calculate the position of the end effector
            end_effector_position = (current_position[0] + current_length * np.cos(current_angle), 
                                     current_position[1] + current_length * np.sin(current_angle))

            # Calculate the Jacobian entries
            J[0, i] = -current_length * np.sin(current_angle)
            J[1, i] = current_length * np.cos(current_angle)

            # Update the current position for the next link
            current_position = end_effector_position

        return J

# Outside the arm class, simple test case function
def run_sim():

    arm = Arm((0, 0))
    arm.add_linkeage(0, 1)
    arm.add_linkeage(np.pi / 2, 1)

    sim = Simulation((1, 1), arm)
    sim.draw(plt)
    plt.axis('equal')
    plt.show()
        
        
