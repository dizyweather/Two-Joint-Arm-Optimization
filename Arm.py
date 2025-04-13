import numpy as np

class Arm:
    # Constructor
    # Could have an empty constrcutor
    def __init__(self, start_position):
        self.start_position = start_position # tuple
        pass

    # Instead have a function that can add lengths and linkeages to an array
    linkeage_angles = [] # in radians (floats)
    linkeage_lengths = [] # in meters or change to like pixels? (floats)

    # Adds a new linkeage to the arrays
    def add_linkeage(self, angle, length):
        self.linkeage_angles.append(angle)
        self.linkeage_lengths.append(length)

    # Based on the linkeages' length and angle, return an x, y of the final/end effector's position
    def calculate_end_position(self):
        pass

    def change_angles(self, array_of_angles):
        self.linkeage_angles = array_of_angles
        pass

    # given a matplot plot, draw yourself on said plot
    def draw(self, plt):
        current_position = self.start_position
        current_position = (0, 0)

        for i in range(len(self.linkeage_angles)):
            current_length = self.linkeage_lengths[i]
            current_angle = self.linkeage_angles[i]

            next_point = (current_position[0] + current_length * np.cos(current_angle), 
                          current_position[1] + current_length * np.sin(current_angle))
            
            x_vals = [current_position[0], next_point[0]]
            y_vals = [current_position[1], next_point[1]]

            print(f"Plotting points {x_vals}, {y_vals}")

            plt.plot(x_vals, y_vals)

            current_position = next_point
        
        
