# -*- coding: utf-8 -*-
#   ___      _ _                    _     
#  / _ \    | | |                  | |    
# / /_\ \ __| | |__   ___  ___  ___| |__  
# |  _  |/ _` | '_ \ / _ \/ _ \/ __| '_ \ 
# | | | | (_| | | | |  __/  __/\__ \ | | |
# \_| |_/\__,_|_| |_|\___|\___||___/_| |_|
# Date:   2020-03-06 12:49:18
# Last Modified time: 2020-03-06 12:55:06

import numpy as np
import matplotlib.pyplot as plt
import math
import sys
sys.path.insert(0,'../')

from a_star import *
from Environment.env import *

grid=[60,60]
start_x=5
start_y=5

goal_x=50
goal_y=20
resolution=1
robot_size=1

env=Environment(grid,resolution,robot_size)
obs_x,obs_y=env.grid_map()

d=A_star(grid,start_x,start_y,goal_x,goal_y,resolution,robot_size)

plt.plot(obs_x, obs_y, ".k")
plt.plot(start_x, start_y, "xr")
plt.plot(goal_x, goal_y, "xb")
plt.grid(True)
plt.axis("equal")
rx,ry=d.algorithm()
plt.plot(rx, ry, "-r")
plt.show()