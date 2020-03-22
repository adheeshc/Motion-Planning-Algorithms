# -*- coding: utf-8 -*-
#   ___      _ _                    _     
#  / _ \    | | |                  | |    
# / /_\ \ __| | |__   ___  ___  ___| |__  
# |  _  |/ _` | '_ \ / _ \/ _ \/ __| '_ \ 
# | | | | (_| | | | |  __/  __/\__ \ | | |
# \_| |_/\__,_|_| |_|\___|\___||___/_| |_|
# Date:   2020-03-22 01:31:43
# Last Modified time: 2020-03-22 01:32:10
from rrt_star import *

if __name__=="__main__":
	print("start " + __file__)
	grid=[30,30]
	
	start_x=0
	start_y=0
	goal_x=5
	goal_y=25

	resolution=1
	robot_size=1
	
	rand_area=[-2,max(grid)]

	rrt=RRT_star(grid,start_x,start_y,goal_x,goal_y,rand_area,expand_dist=0.5,sample_rate=20)
	rrt.plot()
