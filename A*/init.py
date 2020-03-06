# -*- coding: utf-8 -*-
#   ___      _ _                    _     
#  / _ \    | | |                  | |    
# / /_\ \ __| | |__   ___  ___  ___| |__  
# |  _  |/ _` | '_ \ / _ \/ _ \/ __| '_ \ 
# | | | | (_| | | | |  __/  __/\__ \ | | |
# \_| |_/\__,_|_| |_|\___|\___||___/_| |_|
# Date:   2020-03-05 15:28:04
# Last Modified time: 2020-03-06 12:44:55

import numpy as np
import matplotlib.pyplot as plt
import math
import sys
sys.path.insert(0,'../')

from aux_fn import *
from Environment.env import *

class A_star():
	def __init__(self,grid_size,start_x,start_y,goal_x,goal_y,obs_x,obs_y,resolution,robot_size):
		self.map_x=grid_size[0]
		self.map_y=grid_size[1]
		self.sx=start_x
		self.sy=start_y
		self.gx=goal_x
		self.gy=goal_y
		self.obs_x=obs_x
		self.obs_y=obs_y
		self.res=resolution
		self.rs=robot_size

	def algorithm(self):
	    
	    n_start = Node(round(self.sx / self.res), round(self.sy / self.res), 0.0, -1)
	    n_goal = Node(round(self.gx / self.res), round(self.gy / self.res), 0.0, -1)

	    grid=[self.map_x,self.map_y]
	    env=Environment(grid,self.res,self.rs)
	    ob_map=env.obstacle_map()
	    motion = motion_model()

	    open_list, close_list = {}, {}
	    open_list[self.calc_index(n_start, self.map_x, 0, 0)] = n_start
	    
	    while True:
	    	c_id = min(open_list, key=lambda o: open_list[o].cost + self.calc_heuristic(n_goal, open_list[o]))
	    	current = open_list[c_id]
	    	
	    	plt.plot(current.x * self.res, current.y * self.res, "xc")
	    	if len(close_list.keys()) % 10 == 0:
	    		plt.pause(0.001)

	    	if current.x == n_goal.x and current.y == n_goal.y:
	    		print("Found goal")
	    		n_goal.ind = current.ind
	    		n_goal.cost = current.cost
	    		break

	    	
	    	del open_list[c_id]			# Remove the item from the open set
	    	
	    	close_list[c_id] = current	# Add it to the closed set

	    	# expand search grid based on motion model
	    	for i, j in enumerate(motion):
	    		node = Node(current.x + motion[i][0],
	    			current.y + motion[i][1],
	    			current.cost + motion[i][2], c_id)
	    		n_id = self.calc_index(node, self.map_x, 0, 0)
	    		if n_id in close_list:
	    			continue

	    		if not self.verify_node(node, ob_map, 0, 0, self.map_x, self.map_y):
	    			continue
	    		
	    		if n_id not in open_list:
	    			open_list[n_id] = node 
	    		else:
	    			if open_list[n_id].cost >= node.cost:
	    				open_list[n_id] = node
	    rx, ry = self.calc_final_path(n_goal, close_list, self.res)
	    return rx, ry

	def calc_index(self,node,xwidth,xmin,ymin):
		return (node.y-ymin)*xwidth + (node.x-xmin)

	def calc_heuristic(self,n1,n2):
		w=1.0
		d=w*math.sqrt((n1.x-n2.x)**2+(n1.y-n2.y)**2)
		return d

	def verify_node(self,node,ob_map,x_min,y_min,x_max,y_max):
		if node.x<x_min:
			return False
		if node.y<y_min:
			return False
		if node.x>=x_max:
			return False
		if node.y>=y_max:
			return False

		if ob_map[node.x][node.y]:
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


if __name__=="__main__":
	grid=[60,60]
	start_x=5
	start_y=5

	goal_x=10
	goal_y=20
	resolution=1
	robot_size=1

	env=Environment(grid,resolution,robot_size)
	obs_x,obs_y=env.grid_map()

	d=A_star(grid,start_x,start_y,goal_x,goal_y,obs_x,obs_y,resolution,robot_size)
	
	plt.plot(obs_x, obs_y, ".k")
	plt.plot(start_x, start_y, "xr")
	plt.plot(goal_x, goal_y, "xb")
	plt.grid(True)
	plt.axis("equal")
	rx,ry=d.algorithm()
	plt.plot(rx, ry, "-r")
	plt.show()