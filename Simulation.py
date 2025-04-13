import numpy as np


# The thing that performs Newton's method on the arm
# and draws
class Simulation:
    # takes in an arm and goal point
    def __init__(self, goal, arm):
        self.goal = goal # (x, t) tuple
        self.arm = arm
        
    # Draw (Takes arm and goal, plots on plt)
    def draw(self, plt):
        self.arm.draw(plt)
        plt.plot(self.goal[0], self.goal[1], 'g*')

    # performs newton's method (Hard part)
    # recalculate arm position
    # returns the change in position maybe?
    def perform_newtons_method():
        pass

    #