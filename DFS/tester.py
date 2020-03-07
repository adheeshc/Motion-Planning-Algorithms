# -*- coding: utf-8 -*-
#   ___      _ _                    _     
#  / _ \    | | |                  | |    
# / /_\ \ __| | |__   ___  ___  ___| |__  
# |  _  |/ _` | '_ \ / _ \/ _ \/ __| '_ \ 
# | | | | (_| | | | |  __/  __/\__ \ | | |
# \_| |_/\__,_|_| |_|\___|\___||___/_| |_|
# Date:   2020-03-06 12:55:29
# Last Modified time: 2020-03-06 23:11:55

import numpy as np
import matplotlib.pyplot as plt
import math
import sys
sys.path.insert(0,'../')

from init import *
from Environment.env import *

grid=[60,60]
start_x=5
start_y=5
goal_x=30
goal_y=20
resolution=1
robot_size=1

d=DFS(grid,start_x,start_y,goal_x,goal_y,resolution,robot_size)
d.plot()