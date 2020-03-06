# -*- coding: utf-8 -*-
#   ___      _ _                    _     
#  / _ \    | | |                  | |    
# / /_\ \ __| | |__   ___  ___  ___| |__  
# |  _  |/ _` | '_ \ / _ \/ _ \/ __| '_ \ 
# | | | | (_| | | | |  __/  __/\__ \ | | |
# \_| |_/\__,_|_| |_|\___|\___||___/_| |_|
# Date:   2020-03-05 15:28:04
# Last Modified time: 2020-03-06 01:40:18

import numpy as np
import sys
sys.path.insert(0,'../')
import matplotlib.pyplot as plt
from aux_fn import *
from Environment.env import *


class Dijkstra():
	def __init__(self,grid_size,start_x,start_y,goal_x,goal_y,resolution,robot_size):
		self.map_x=grid_size[0]
		self.map_y=grid_size[1]
		self.sx=start_x
		self.sy=start_y
		self.gx=goal_x
		self.gy=goal_y
		
		self.res=resolution
		self.rs=robot_size

		self.pre_processing()
	
	def pre_processing(self): #
		self.n_start=Node(round(self.sx/self.res),round(self.sy/self.res),0,-1)
		self.n_goal=Node(round(self.gx/self.res),round(self.gy/self.res),0,-1)
		
		grid=[self.map_x,self.map_y]
		env=Environment(grid,self.res,self.rs)
		#self.ox,self.oy=env.grid_map()
		self.ob_map=env.obstacle_map()
		self.motion=motion_model()
		
	def algorithm(self):
		open_list,close_list={},{}
		open_list[self.calc_index(self.n_start,self.map_x,0,0)] = self.n_start

		
		while True:
			c_id=min(open_list,key=lambda o:open_list[o].cost)	#Find node of min cost
			print('a')
			current=open_list[c_id]								

			plt.plot(current.x*self.res,current.y*self.res,"xc")
			if len(close_list.keys())%10==0:
				plt.pause(0.001)

			if current.x==self.n_goal.x and current.y==self.n_goal.y:		#Check if goal reached
				print("found goal")
				self.n_goal.ind=current.ind
				self.n_goal.cost=current.cost
				break

			del open_list[c_id]			# Remove the item from the open list
			close_list[c_id]=current	# Add it to the closed list

			for i,j in enumerate(self.motion):
				node=Node(current.x+self.motion[i][0],
							current.y+self.motion[i][1],
							current.cost+self.motion[i][2],
							c_id)
				n_id=self.calc_index(node,self.map_x,0,0)

				if not self.verify_node(node,self.ob_map,0,0,self.map_x,self.map_y):	#CHECK COLLISION AND BOUNDARIES
					continue

				if n_id in close_list:
					continue

				if n_id in open_list:
					if open_list[n_id].cost>node.cost:
						open_list[n_id].cost=node.cost
						open_list[n_id].ind=c_id
				else:
					open_list[n_id]=node

		rx,ry=self.calc_final_path(self.n_goal,close_list,self.res)
		
		return rx,ry

	def calc_index(self,node, xwidth, xmin, ymin):
		return (node.y - ymin) * xwidth + (node.x - xmin)

	def verify_node(self,node,ob_map,xmin,ymin,xmax,ymax):
		if node.x<xmin:
			return False
		if node.y<ymin:
			return False
		if node.x>xmax:
			return False
		if node.y>ymax:
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
	grid=[30,30]
	start_x=0
	start_y=0

	goal_x=20
	goal_y=20
	resolution=1
	robot_size=1

	d=Dijkstra(grid,start_x,start_y,goal_x,goal_y,resolution,robot_size)
	rx,ry=d.algorithm()
	plt.plot(env.obs_x, env.obs_y, "k")
	plt.scatter(self.start_point[0],self.start_point[1],c='g')
	plt.scatter(self.goal_point[0],self.goal_point[1],c='r')
	plt.plot(rx, ry, "-r")
	# plt.plot(sx, sy, "xr")
	# plt.plot(gx, gy, "xb")
	plt.grid(True)
	plt.axis("equal")
	plt.show()
