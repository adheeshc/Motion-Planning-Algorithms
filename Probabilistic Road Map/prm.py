# -*- coding: utf-8 -*-
#   ___      _ _                    _     
#  / _ \    | | |                  | |    
# / /_\ \ __| | |__   ___  ___  ___| |__  
# |  _  |/ _` | '_ \ / _ \/ _ \/ __| '_ \ 
# | | | | (_| | | | |  __/  __/\__ \ | | |
# \_| |_/\__,_|_| |_|\___|\___||___/_| |_|
# Date:   2020-03-05 15:28:04
# Last Modified time: 2020-03-24 02:43:58

import numpy as np
import matplotlib.pyplot as plt
import random
import math
import copy

import sys
sys.path.insert(0,'../')

from aux_fn import Node,KDTree
from Environment.env import *

class PRM():

	def __init__(self, grid_size,start_x, start_y,goal_x,goal_y, res=1,robot_size=1, n_samples=500,n_nearest_nodes=10,max_edge_length=30):
		self.map_x=grid_size[0]
		self.map_y=grid_size[1]
		self.sx=start_x
		self.sy=start_y
		self.gx=goal_x
		self.gy=goal_y
		self.res=res
		self.rs=robot_size

		self.n_samples=n_samples
		self.n_nearest_nodes=n_nearest_nodes
		self.max_edge_length=max_edge_length

		self.env=Environment(grid_size)
		self.obs_x,self.obs_y=self.env.grid_map()

	def algorithm(self,):
		obstacles=np.vstack((self.obs_x,self.obs_y)).T
		ob_tree=KDTree(obstacles)

		sample_x,sample_y=self.sample_points(self.sx,self.sy,self.gx,self.gy,self.rs,self.obs_x,self.obs_y,ob_tree)
		plt.plot(sample_x, sample_y, ".b")

		road_map=self.generate_roadmap(sample_x,sample_y,self.rs,ob_tree)

		rx,ry=self.dijkstra(self.sx,self.sy,self.gx,self.gy,self.obs_x,self.obs_y,self.rs,road_map,sample_x,sample_y)

		return rx,ry

	def dijkstra(self,sx, sy, gx, gy, obs_x, obs_y, rs, road_map, sample_x, sample_y,show_animation=True):
		n_start = Node(round(self.sx / self.res), round(self.sy / self.res), 0.0, -1)
		n_goal = Node(round(self.gx / self.res), round(self.gy / self.res), 0.0, -1)

		open_list, closed_list = {}, {}
		open_list[len(road_map) - 2] = n_start

		while True:
			if not open_list:
				print("Cannot find path")
				break

			c_id = min(open_list, key=lambda o: open_list[o].cost)
			current = open_list[c_id]

			plt.plot(current.x*self.res, current.y*self.res, "oy",zorder=1)
			if len(closed_list.keys()) % 2 == 0:
				plt.pause(0.001)

			if c_id == (len(road_map) - 1):
				print("Found goal!")
				n_goal.ind = current.ind
				n_goal.cost = current.cost
				break

			
			del open_list[c_id]
			
			closed_list[c_id] = current

			
			for i in range(len(road_map[c_id])):
				n_id = road_map[c_id][i]
				dx = sample_x[n_id] - current.x
				dy = sample_y[n_id] - current.y
				d = math.sqrt(dx**2 + dy**2)
				node = Node(sample_x[n_id], sample_y[n_id],current.cost + d, c_id)

				if n_id in closed_list:
					continue
				
				if n_id in open_list:
					if open_list[n_id].cost > node.cost:
						open_list[n_id].cost = node.cost
						open_list[n_id].pind = c_id
				else:
					open_list[n_id] = node

		
		rx, ry =  self.calc_final_path(n_goal,closed_list,self.res)
		return rx, ry

	def sample_points(self,sx,sy,gx,gy,rs,obs_x,obs_y,ob_tree):

		sample_x,sample_y=[],[]
		
		while len(sample_x)<=self.n_samples:
			temp_x=(random.random()-min(obs_x))*(max(obs_x)-min(obs_x))
			temp_y=(random.random()-min(obs_y))*(max(obs_y)-min(obs_y))

			dist,index=ob_tree.search_tree(np.array([temp_x,temp_y]).reshape(2,1))

			if dist[0]>=rs:
				sample_x.append(temp_x)
				sample_y.append(temp_y)

		sample_x.append(sx)
		sample_y.append(sy)
		sample_x.append(gx)
		sample_y.append(gy)

		return sample_x,sample_y

	def generate_roadmap(self,sample_x,sample_y,rs,ob_tree):
		road_map=[]
		samples=np.vstack((sample_x,sample_y)).T
		sample_tree=KDTree(samples)
		
		for (i,ix,iy) in zip(range(len(sample_x)),sample_x,sample_y):
			dist,index=sample_tree.search_tree(np.array([ix,iy]).reshape(2,1),k=len(sample_x))
			ind=index[0]
			edge_id=[]
			
			for j in range(1,len(ind)):
				nx=sample_x[ind[j]]
				ny=sample_y[ind[j]]

				if not self.verify_node(ix,iy,nx,ny,rs,ob_tree):
					edge_id.append(ind[j])
				#edge_id.append(ind[j])

				if len(edge_id)>=self.n_nearest_nodes:
					break

			road_map.append(edge_id)

		return road_map

	def verify_node(self,sx,sy,gx,gy,rs,ob_tree):
		
		dx=gx-sx
		dy=gy-sy
		theta=math.atan2(dy,dx)
		d=math.sqrt(dx**2+dy**2)

		if d>=self.max_edge_length:
			return True

		n_steps=round((d/rs)*self.res)

		for i in range(n_steps):
			dist,index=ob_tree.search_tree(np.array([sx,sy]).reshape(2,1)) 
			if dist[0]<=rs:
				return True

			sx+=rs*math.cos(theta)
			sy+=rs*math.sin(theta)

		dist,index=ob_tree.search_tree(np.array([gx,gy]).reshape(2,1)) #check goal
		if dist[0]<=rs:
			return True

		return False



		return True

	def calc_final_path(self,n_goal,close_list,res):
		rx,ry=[n_goal.x*res],[n_goal.y*res]
		ind=n_goal.ind
		while ind!=-1:
			n=close_list[ind]
			rx.append(n.x*res)
			ry.append(n.y*res)
			ind=n.ind
		return rx,ry


	def plot(self,record=False):
		plt.plot(self.obs_x, self.obs_y, ".k")
		plt.plot(self.sx, self.sy, "og",zorder=4)
		plt.plot(self.gx, self.gy, "or",zorder=3)
		plt.grid(True)
		plt.axis("equal")
		plt.title("Probabilistic Road Map")
		if record:
			plt.pause(5)
		rx,ry=self.algorithm()
		plt.plot(rx, ry, "-b",zorder=2)
		plt.show()