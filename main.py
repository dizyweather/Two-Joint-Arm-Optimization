
from Arm import Arm
import matplotlib.pyplot as plt
import numpy as np

hi = (0, 0)
hi = (hi[0] + 1, hi[1])

arm = Arm((0, 0))
arm.add_linkeage(0, 1)
arm.add_linkeage(np.pi /2, 1)

arm.change_angles([np.pi, np.pi])
arm.draw(plt)

plt.xticks(range(-5, 5))
plt.yticks(range(-1, 5))

plt.show()


print(np.sin(np.pi))