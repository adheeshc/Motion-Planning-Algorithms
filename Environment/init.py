# -*- coding: utf-8 -*-
#   ___      _ _                    _     
#  / _ \    | | |                  | |    
# / /_\ \ __| | |__   ___  ___  ___| |__  
# |  _  |/ _` | '_ \ / _ \/ _ \/ __| '_ \ 
# | | | | (_| | | | |  __/  __/\__ \ | | |
# \_| |_/\__,_|_| |_|\___|\___||___/_| |_|
# Date:   2020-03-05 15:28:04
# Last Modified time: 2020-03-05 17:11:52

import numpy as np
import math
import copy
import matplotlib.pyplot as plt

class Environment():
	def __init__(self,grid,res,rob_size):
		self.map_x=grid[0]
		self.map_y=grid[1]
		self.grid_size=res
		self.robot_size=rob_size
		self.start_point=[5,4]
		self.goal_point=[self.map_x-5,self.map_y-4]


	def grid_map(self):
		obs_x, obs_y = [], []

		for i in range(self.map_x):
			obs_x.append(i)
			obs_y.append(0)

			obs_x.append(i)
			obs_y.append(self.map_y)

		for j in range(self.map_y):
			obs_x.append(0)
			obs_y.append(j)

			obs_x.append(self.map_x)
			obs_y.append(j)

		if self.map_x>self.map_y:
			for i in range(int(self.map_x*(1/3))):
				obs_x.append(self.map_x*(1/3))
				obs_y.append(i)
			for j in range(int(self.map_x*(1/3))):
			 	obs_x.append(self.map_x*(2/3))
			 	obs_y.append(self.map_y-j)
		elif self.map_x==self.map_y:
			for i in range(int(self.map_x*(1/2))):
				obs_x.append(self.map_x*(1/3))
				obs_y.append(i)
			for j in range(int(self.map_x*(1/2))):
			 	obs_x.append(self.map_x*(2/3))
			 	obs_y.append(self.map_y-j)
		else:
			for i in range(int(self.map_y*(1/3))):
				obs_x.append(self.map_x*(1/3))
				obs_y.append(i)
			for j in range(int(self.map_y*(1/3))):
			 	obs_x.append(self.map_x*(2/3))
			 	obs_y.append(self.map_y-j)
		return obs_x,obs_y

	def show_map(self):
		obs_x,obs_y=self.grid_map()
		plt.scatter(obs_x,obs_y,c='k')
		plt.scatter(self.start_point[0],self.start_point[1],c='g')
		plt.scatter(self.goal_point[0],self.goal_point[1],c='r')
		#plt.scatter(self.path,c='y')
		plt.show()
