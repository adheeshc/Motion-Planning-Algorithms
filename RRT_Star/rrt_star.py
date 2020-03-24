# -*- coding: utf-8 -*-
#   ___      _ _                    _     
#  / _ \    | | |                  | |    
# / /_\ \ __| | |__   ___  ___  ___| |__  
# |  _  |/ _` | '_ \ / _ \/ _ \/ __| '_ \ 
# | | | | (_| | | | |  __/  __/\__ \ | | |
# \_| |_/\__,_|_| |_|\___|\___||___/_| |_|
# Date:   2020-03-05 15:28:04
# Last Modified time: 2020-03-24 05:09:53

import numpy as np
import matplotlib.pyplot as plt
import random
import math
import copy

import sys
sys.path.insert(0,'../')

from aux_fn import *
from Environment.env import *


class RRT_star():
	def __init__(self, grid_size,start_x, start_y,goal_x,goal_y,randArea, res=1,robot_size=1, expand_dist=0.5, sample_rate=20, max_iter=100):
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

	def algorithm(self, animation=True):
		
		n_start = Node(round(self.sx / self.res), round(self.sy / self.res))
		n_goal = Node(round(self.gx / self.res), round(self.gy / self.res))
		
		self.node_list=[n_start] 
		self.ob_map=self.env.obstacle_wall()
		
		for i in range(self.max_iter):
			#RANDOM SAMPLING
			rnd = self.get_random_point()

			#FIND NEAREST NODE BASED ON RND
			n_ind = self.calc_index(self.node_list, rnd)

			#EXPAND TREE
			nearest_node=self.node_list[n_ind]
			theta=math.atan2(rnd[1]-nearest_node.y,rnd[0]-nearest_node.x)

			current=Node(rnd[0],rnd[1])
			current_dist=np.sqrt((rnd[1]-nearest_node.y)**2+(rnd[0]-nearest_node.x)**2)
			if current_dist<=self.expand_dist:
				pass
			else:
				current.x=nearest_node.x+self.expand_dist*math.cos(theta)
				current.y=nearest_node.y+self.expand_dist*math.sin(theta)
			current.ind=None
			current.cost=float("inf")

			if self.verify_node(current):
				near_inds = self.find_near_nodes(current)
				current = self.choose_ind(current, near_inds)
				self.node_list.append(current)
				self.rewire(current, near_inds)

				#FIND GOAL
			dx=current.x-self.gx
			dy=current.y-self.gy
			d=math.sqrt(dx*dx+dy*dy)
			if d<=self.expand_dist:
				print("Found goal")
				break

			#PLOT
			#rnd_pts,=plt.plot(rnd[0], rnd[1], "^k")
			# rnd_pts.set_visible(False)
			for node in self.node_list:
				if node.ind is not None:
					plt.plot([node.x, self.node_list[node.ind].x],[node.y, self.node_list[node.ind].y], "-y",zorder=1)					
			if i%5==0:
				plt.pause(0.01)

		self.calc_final_path(n_start,n_goal)
		
		return self.path

	def calc_index(self, node_list, rnd):
		dist_list=[(node.x-rnd[0])**2+(node.y-rnd[1])**2 for node in node_list]
		min_ind=dist_list.index(min(dist_list))
		return min_ind

	def verify_node(self, node):
		if node.x<0:
			return False
		if node.y<0:
			return False
		if node.x>=self.map_x:
			return False
		if node.y>=self.map_y:
			return False

		for ox,oy in zip(self.obs_x,self.obs_y):
			dx=node.x-ox
			dy=node.y-oy
			d=np.sqrt(dx**2 + dy**2)
			if ox>0 and oy>0:
				if d<1:
					return False

		return True  # safe

	def get_random_point(self):
		if random.randint(0, 100) > self.sample_rate:
			rnd=[random.uniform(self.min_rand,self.max_rand),random.uniform(self.min_rand,self.max_rand)]
		else:
			rnd = [self.gx,self.gy]
		return rnd

	def choose_ind(self, current, near_inds):
		if not near_inds:
			return current

		cost_list = []
		for i in near_inds:
			dx=current.x-self.node_list[i].x
			dy=current.y-self.node_list[i].y
			d=math.sqrt(dx**2+dy**2)
			theta=math.atan2(dy,dx)
			if self.verify_secondary_node(self.node_list[i], theta, d):
				cost_list.append(self.node_list[i].cost + d)
			else:
				cost_list.append(float("inf"))

		min_cost = min(cost_list)
		min_ind = near_inds[cost_list.index(min_cost)]

		if min_cost == float("inf"):
			print("min cost is inf")
			return current

		current.cost = min_cost
		current.ind = min_ind

		return current
		

	def find_near_nodes(self, current):
		n_node = len(self.node_list)
		r = 50.0*math.sqrt((math.log(n_node)/n_node))
		dist_list=[(node.x-current.x)**2+(node.y-current.y)**2 for node in self.node_list]
		near_inds=[dist_list.index(i) for i in dist_list if i<=r**2]
		return near_inds

	def rewire(self, current, near_inds):
		n_node = len(self.node_list)
		for i in near_inds:
			dx=current.x-self.node_list[i].x
			dy=current.y-self.node_list[i].y
			d=math.sqrt(dx**2+dy**2)

			if self.node_list[i].cost>current.cost+d:
				theta = math.atan2(dy, dx)
				if self.verify_secondary_node(self.node_list[i], theta, d):
					self.node_list[i].ind=n_node-1
					self.node_list[i].cost=current.cost+d

	def verify_secondary_node(self, near_node, theta, d):
		temp = copy.deepcopy(near_node)
		for i in range(int(d/self.expand_dist)):
			temp.x+=self.expand_dist*math.cos(theta)
			temp.y+=self.expand_dist*math.sin(theta)
			if not self.verify_node(temp):
				return False

		return True

	def calc_final_path(self,n_start,n_goal):
		self.path = [[n_goal.x, n_goal.y]]
		lastIndex = len(self.node_list) - 1
		while self.node_list[lastIndex].ind is not None:
			node = self.node_list[lastIndex]
			self.path.append([node.x, node.y])
			lastIndex = node.ind
		self.path.append([n_start.x, n_start.y])
		return self.path

	def plot(self,record=False):
		plt.clf()
		plt.axis("equal")
		plt.grid(True)
		plt.title("RRT*")
		plt.plot(self.sx, self.sy, "og",zorder=4)
		plt.plot(self.gx, self.gy, "or",zorder=3)
		plt.plot(self.obs_x, self.obs_y, ".k")
		if record:
			plt.pause(5)
		path=self.algorithm()
		plt.plot([x for (x, y) in path], [y for (x, y) in path], '-b')
		plt.show()