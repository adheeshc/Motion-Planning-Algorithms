# -*- coding: utf-8 -*-
#   ___      _ _                    _     
#  / _ \    | | |                  | |    
# / /_\ \ __| | |__   ___  ___  ___| |__  
# |  _  |/ _` | '_ \ / _ \/ _ \/ __| '_ \ 
# | | | | (_| | | | |  __/  __/\__ \ | | |
# \_| |_/\__,_|_| |_|\___|\___||___/_| |_|
# Date:   2020-03-05 15:28:04
# Last Modified time: 2020-03-21 19:57:35

import numpy as np
import matplotlib.pyplot as plt
import random
import math
import copy

import sys
sys.path.insert(0,'../')

from aux_fn import *
from Environment.env import *

class RRT():

	def __init__(self, grid_size,start_x, start_y,goal_x,goal_y,randArea, res=1,robot_size=1, expand_dist=1.0, sample_rate=5, max_iter=500):
		self.map_x=grid_size[0]
		self.map_y=grid_size[1]
		self.sx=start_x
		self.sy=start_y
		self.gx=goal_x
		self.gy=goal_y
		self.res=res
		self.rs=robot_size

		self.min_rand = randArea[0]
		self.max_rand = randArea[1]
		self.expand_dist=expand_dist
		self.sample_rate=sample_rate
		self.max_iter=max_iter
		
		self.env=Environment(grid_size)
		self.obs_x,self.obs_y=self.env.grid_map()

	def algorithm(self, display=False):

		n_start = Node(round(self.sx / self.res), round(self.sy / self.res))
		n_goal = Node(round(self.gx / self.res), round(self.gy / self.res))

		self.node_list=[n_start] 
		ob_map=self.env.obstacle_wall()
		i=0
		while True:
			i+=1
			#RANDOM SAMPLING
			if random.randint(0,100)>self.sample_rate:
				rnd=[random.uniform(self.min_rand,self.max_rand),random.uniform(self.min_rand,self.max_rand)]
			else:
				rnd=[self.gx,self.gy]

			#FIND NEAREST NODE BASED ON RND
			n_ind=self.calc_index(self.node_list,rnd)

			#EXPAND TREE
			nearest_node=self.node_list[n_ind]
			theta=math.atan2(rnd[1]-nearest_node.y,rnd[0]-nearest_node.x)

			current=copy.deepcopy(nearest_node)
			current.x+=self.expand_dist*math.cos(theta)
			current.y+=self.expand_dist*math.sin(theta)
			current.ind=n_ind

			# HIT OBSTACLES
			if not self.verify_node(current,ob_map,0,0,self.map_x,self.map_y):
				continue

			#ADD NODE TO LIST
			self.node_list.append(current)
			print(f"nodes:{len(self.node_list)-1}")
			if len(self.node_list)==1000:
				break

			#FIND GOAL
			dx=current.x-self.gx
			dy=current.y-self.gy
			d=math.sqrt(dx*dx+dy*dy)
			if d<=self.expand_dist:
				print("Found goal")
				break

			#PLOT
			# rnd_pts,=plt.plot(rnd[0], rnd[1], "^k")
			# rnd_pts.set_visible(False)
			for node in self.node_list:
				if node.ind is not None:
					plt.plot([node.x, self.node_list[node.ind].x],[node.y, self.node_list[node.ind].y], "-y",zorder=1)
			if len(self.node_list)%1==0:
				plt.pause(0.1)
		
		#BACKTRACK TO FIND PATH			
		self.calc_final_path(n_start,n_goal)
		return self.path

	def calc_index(self,node_list,rnd):
		dist_list=[(node.x-rnd[0])**2 + (node.y-rnd[1])**2 for node in node_list]
		min_ind=dist_list.index(min(dist_list))
		return min_ind
	
	def verify_node(self, node,ob_map, xmin,ymin,xmax,ymax):
		if node.x<xmin:
			return False
		if node.y<ymin:
			return False
		if node.x>=xmax:
			return False
		if node.y>=ymax:
			return False

		for ox,oy in zip(self.obs_x,self.obs_y):
			dx=node.x-ox
			dy=node.y-oy
			d=np.sqrt(dx**2 + dy**2)
			if ox>0 and oy>0:
				if d<1:
					return False

		return True  # safe

	def calc_final_path(self,n_start,n_goal):
		self.path = [[n_goal.x, n_goal.y]]
		lastIndex = len(self.node_list) - 1
		while self.node_list[lastIndex].ind is not None:
			node = self.node_list[lastIndex]
			self.path.append([node.x, node.y])
			lastIndex = node.ind
		self.path.append([n_start.x, n_start.y])
		return self.path

	def plot(self, rnd=None):
		plt.clf()
		plt.axis("equal")
		plt.grid(True)
		
		plt.plot(self.sx, self.sy, "og",zorder=4)
		plt.plot(self.gx, self.gy, "or",zorder=3)
		plt.plot(self.obs_x, self.obs_y, ".k")

		path=self.algorithm()
		plt.plot([x for (x, y) in path], [y for (x, y) in path], '-b')
		plt.show()