# -*- coding: utf-8 -*-
#   ___      _ _                    _     
#  / _ \    | | |                  | |    
# / /_\ \ __| | |__   ___  ___  ___| |__  
# |  _  |/ _` | '_ \ / _ \/ _ \/ __| '_ \ 
# | | | | (_| | | | |  __/  __/\__ \ | | |
# \_| |_/\__,_|_| |_|\___|\___||___/_| |_|
# Date:   2020-03-21 19:56:54
# Last Modified time: 2020-03-21 19:57:26
from rrt import *


if __name__=="__main__":
	print("start " + __file__)
	grid=[20,20]
	start_x=0
	start_y=0

	goal_x=15
	goal_y=10
	resolution=1
	robot_size=1
	rand_area=[-2,15]
	show_display = True
	rrt=RRT(grid,start_x,start_y,goal_x,goal_y,rand_area)
	rrt.plot()