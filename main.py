
from Arm import Arm
from Simulation import Simulation
import matplotlib.pyplot as plt
import numpy as np

arm = Arm((0, 0))
arm.add_linkeage(0, 1)
arm.add_linkeage(np.pi /2, 1)

sim = Simulation((1, 1), arm)
sim.draw(plt)

plt.show()

