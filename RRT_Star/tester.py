# -*- coding: utf-8 -*-
#   ___      _ _                    _     
#  / _ \    | | |                  | |    
# / /_\ \ __| | |__   ___  ___  ___| |__  
# |  _  |/ _` | '_ \ / _ \/ _ \/ __| '_ \ 
# | | | | (_| | | | |  __/  __/\__ \ | | |
# \_| |_/\__,_|_| |_|\___|\___||___/_| |_|
# Date:   2020-03-22 01:31:43
# Last Modified time: 2020-03-24 05:15:31
from rrt_star import *

if __name__=="__main__":
	print("Start " + __file__)
	grid=[10,10]

	start_x=1
	start_y=1
	goal_x=8
	goal_y=4

	resolution=1
	robot_size=1
	
	rand_area=[-2,15]

	rrt=RRT_star(grid,start_x,start_y,goal_x,goal_y,rand_area,expand_dist=1,max_iter=1000)
	rrt.plot(record=True)