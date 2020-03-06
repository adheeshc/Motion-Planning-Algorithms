# -*- coding: utf-8 -*-
#   ___      _ _                    _     
#  / _ \    | | |                  | |    
# / /_\ \ __| | |__   ___  ___  ___| |__  
# |  _  |/ _` | '_ \ / _ \/ _ \/ __| '_ \ 
# | | | | (_| | | | |  __/  __/\__ \ | | |
# \_| |_/\__,_|_| |_|\___|\___||___/_| |_|
# Date:   2020-03-05 15:28:04
# Last Modified time: 2020-03-05 18:56:24

import numpy as np



class A_star():
	def __init__(self,grid_size,start_x,start_y,goal_x,goal_y,obs_x,obs_y,resolution,robot_size):
		self.map_x=grid[0]
		self.map_y=grid[1]
		self.sx=start_x
		self.sy=start_y
		self.gx=goal_x
		self.gy=goal_y
		self.ox=obs_x
		self.oy=obs_y
		self.res=resolution
		self.rs=robot_size


	def algo():
		grid=[self.map_x,self.map_y]
		env=Environment(grid,self.res,self.rs)
		ob_map=env.obstacle_map()