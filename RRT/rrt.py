# -*- coding: utf-8 -*-
#   ___      _ _                    _     
#  / _ \    | | |                  | |    
# / /_\ \ __| | |__   ___  ___  ___| |__  
# |  _  |/ _` | '_ \ / _ \/ _ \/ __| '_ \ 
# | | | | (_| | | | |  __/  __/\__ \ | | |
# \_| |_/\__,_|_| |_|\___|\___||___/_| |_|
# Date:   2020-03-05 15:28:04
# Last Modified time: 2020-03-15 21:39:36

import numpy as np
import matplotlib.pyplot as plt
import math
import random
import copy
import sys
sys.path.insert(0,'../')

from aux_fn import *
from Environment.env import *

class RRT():
	def __init__(self,grid_size,start_x,start_y,goal_x,goal_y,rand_area,resolution=1,robot_size=1,expand_dist=1,sample_rate=5,max_iter=500):
		self.map_x=grid_size[0]
		self.map_y=grid_size[1]
		self.sx=start_x
		self.sy=start_y
		self.gx=goal_x
		self.gy=goal_y
		self.res=resolution
		self.rs=robot_size
		self.min_rand=rand_area[0]
		self.max_rand=rand_area[1]
		self.expand_dist=expand_dist
		self.sample_rate=sample_rate
		self.max_iter=max_iter
		self.env=Environment(grid_size,self.res,self.rs)
		self.obs_x,self.obs_y=self.env.grid_map()

	def algorithm(self):
		
		n_start = Node(round(self.sx / self.res), round(self.sy / self.res))
		n_goal = Node(round(self.gx / self.res), round(self.gy / self.res))
		
		self.node_list=[n_start]

		
		ob_map=self.env.obstacle_wall()

		motion = motion_model()

		i=0
		while True:
			i+=1

			#RADNDOM SAMPLE 
			if random.randint(0,100)>self.sample_rate:
				rnd=[random.uniform(self.min_rand,self.max_rand),random.uniform(self.min_rand,self.max_rand)]
			else:
				rnd=[self.gx,self.gy]

			#FIND NEAREST NODE BASED ON RANDOM
			n_ind=self.calc_index(self.node_list,rnd)
			if i%30	==0:
				break
			
			#EXPAND TREE
			nearest_node=self.node_list[n_ind]
			theta=math.atan2(rnd[1]-nearest_node.y,rnd[1]-nearest_node.x)

			current=copy.deepcopy(nearest_node)
			current.x+=int(self.expand_dist*math.cos(theta))
			current.y+=int(self.expand_dist*math.sin(theta))
			current.ind=n_ind

			#CHECK COLLISIONS
			# if not self.verify_node(current,ob_map,self.obs_x,self.obs_y):
			# 	continue

			#ADD NODE TO LIST
			self.node_list.append(current)
			print(f'size of node list: {len(self.node_list)}')

			#CHECK GOAL
			if current.x == n_goal.x and current.y == n_goal.y:
				print("Found goal")
				break
		rx, ry = self.calc_final_path(n_goal, self.node_list, self.res)
		return rx,ry


	def calc_index(self,node_list,rnd):
		dist_list=[(node.x-rnd[0])**2 + (node.y-rnd[1])**2 for node in node_list]
		min_ind=dist_list.index(min(dist_list))
		return min_ind

	def verify_node(self,node,ob_map,obs_x,obs_y):
		
		if node.x<0:
			return False
		if node.y<0:
			return False
		if node.x>=self.map_x:
			return False
		if node.y>=self.map_y:
			return False

		if ob_map[node.x][node.y]:
			return False
		return True


	def calc_final_path(self,n_goal,node_list,res):
		rx,ry=[n_goal.x*res],[n_goal.y*res]
		ind=len(self.node_list)-1
		while ind!=-1:
			n=node_list[ind]
			rx.append(n.x*res)
			ry.append(n.y*res)
			ind=n.ind
		return rx,ry

	def plot(self):
		plt.plot(self.obs_x, self.obs_y, ".k")
		plt.plot(self.sx, self.sy, "og",zorder=4)
		plt.plot(self.gx, self.gy, "or",zorder=3)
		plt.grid(True)
		plt.axis("equal")
		rx,ry=self.algorithm()
		print(rx)
		print(ry)
		plt.plot(rx, ry, "-b",zorder=2)
		plt.show()

if __name__=="__main__":

	grid=[60,60]
	start_x=5
	start_y=5

	goal_x=50
	goal_y=20
	resolution=1
	robot_size=1
	rand_area=[-2,15]
	env=Environment(grid,resolution,robot_size)
	obs_x,obs_y=env.grid_map()

	rrt=RRT(grid,start_x,start_y,goal_x,goal_y,rand_area)
	rrt.plot()