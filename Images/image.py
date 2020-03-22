# -*- coding: utf-8 -*-
#   ___      _ _                    _     
#  / _ \    | | |                  | |    
# / /_\ \ __| | |__   ___  ___  ___| |__  
# |  _  |/ _` | '_ \ / _ \/ _ \/ __| '_ \ 
# | | | | (_| | | | |  __/  __/\__ \ | | |
# \_| |_/\__,_|_| |_|\___|\___||___/_| |_|
# Date:   2020-03-22 03:51:43
# Last Modified time: 2020-03-22 04:09:35


import sys
sys.path.insert(0,'../')

from RRT.rrt import *
from Environment.env import *
grid=[20,20]
start_x=1
start_y=1

goal_x=19
goal_y=19
resolution=1
robot_size=1
rand_area=[-2,15]
show_display = True
rrt=RRT(grid,start_x,start_y,goal_x,goal_y,rand_area,expand_dist=1)
rrt.plot(True)