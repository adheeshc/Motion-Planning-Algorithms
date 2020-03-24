# -*- coding: utf-8 -*-
#   ___      _ _                    _     
#  / _ \    | | |                  | |    
# / /_\ \ __| | |__   ___  ___  ___| |__  
# |  _  |/ _` | '_ \ / _ \/ _ \/ __| '_ \ 
# | | | | (_| | | | |  __/  __/\__ \ | | |
# \_| |_/\__,_|_| |_|\___|\___||___/_| |_|
# Date:   2020-03-05 15:28:04
# Last Modified time: 2020-03-24 05:13:43

import numpy as np
import math
import copy
import matplotlib.pyplot as plt

class Environment():
	def __init__(self,grid,resolution=1,robot_size=1):
		self.map_x=grid[0]
		self.map_y=grid[1]
		self.res=resolution
		self.rs=robot_size
		self.start_point=[5,4]
		self.goal_point=[self.map_x-5,self.map_y-4]

	def grid_map(self):
		val=30
		val2=50
		obs_x, obs_y = [], []
		for i in range(0,val2):	#lower and upper Walls
			obs_y.append(0)
			obs_y.append(self.map_y)
		temp=np.linspace(0,self.map_x,num=val2)
		for i in temp:
			obs_x.append(i)	
		temp=np.linspace(0,self.map_x,num=val2)
		for i in temp:
			obs_x.append(i)
			
		for j in range(0,val2):	#left and right walls
			obs_x.append(0)
			obs_x.append(self.map_x)
		
		temp=np.linspace(0,self.map_y,num=val2)
		for i in temp:
			obs_y.append(i)
		
		temp=np.linspace(0,self.map_y,num=val2)
		for i in temp:
			obs_y.append(i)
			
		if self.map_x>self.map_y:
			for i in range(0,val):	#first wall
				obs_x.append(self.map_x*(1/3))
			temp=np.linspace(0,self.map_x*(1/3),num=val)
			for i in temp:
				obs_y.append(i)

			for j in range(0,val):	#second wall
			 	obs_x.append(self.map_x*(2/3))
			temp=np.linspace(self.map_y,self.map_y-self.map_y/3,num=val)
			for i in temp:
				obs_y.append(i)

		elif self.map_x==self.map_y:
			for i in range(0,val):	#first wall
				obs_x.append(self.map_x*(1/3))
			temp=np.linspace(0,self.map_x*(2/3),num=val)
			for i in temp:
				obs_y.append(i)
			for j in range(0,val):	#second wall
			 	obs_x.append(self.map_x*(2/3))
			temp=np.linspace(self.map_y,self.map_y-self.map_y*2/3,num=val)
			for i in temp:
				obs_y.append(i)

		else:
			for i in range(0,val):	#first wall
				obs_x.append(self.map_x*(1/3))
			temp=np.linspace(0,self.map_x*(1/3),num=val)
			for i in temp:
				obs_y.append(i)

			for j in range(0,val):	#second wall
			 	obs_x.append(self.map_x*(2/3))
			temp=np.linspace(self.map_y,self.map_y-self.map_y/3,num=val)
			for i in temp:
				obs_y.append(i)
	
		return obs_x,obs_y
	
	def obstacle_wall(self):

		obs_x,obs_y=self.grid_map()
		
		obs_x = [i / self.res for i in obs_x]
		obs_y = [i / self.res for i in obs_y]

		ob_map=[[False for i in range(self.map_y)] for i in range(self.map_x)]

		for i in range(0,self.map_x):
			for j in range(0,self.map_y):
				for a,b in zip(obs_x,obs_y):
					d=math.sqrt((a-i)**2+(b-j)**2)
					if d<=self.rs/self.res:
						ob_map[i][j]=True
						#ob_map[i][j]=0
						break
		return ob_map
	
	def obstacle_objects(self):
		self.obstacle_list = [(5, 5, 1),
						(3, 6, 2),
						(3, 8, 2),
						(3, 10, 2),
						(7, 5, 2),
						(9, 5, 2)]  # [x,y,size(radius)]
		return self.obstacle_list

	def show_map(self):
		obs_x,obs_y=self.grid_map()
		ob_map=self.obstacle_wall()
		
		#plt.scatter(obs_x,obs_y,c='k')
		# plt.scatter(self.start_point[0],self.start_point[1],c='g')
		# plt.scatter(self.goal_point[0],self.goal_point[1],c='r')
		ob_obj=self.obstacle_objects()
		
		for (ox, oy, size) in ob_obj:
			plt.plot(ox, oy, "ok", ms=30 * size)
		plt.axis([-2,15,-2,15])
		plt.show()
