# -*- coding: utf-8 -*-
#   ___      _ _                    _     
#  / _ \    | | |                  | |    
# / /_\ \ __| | |__   ___  ___  ___| |__  
# |  _  |/ _` | '_ \ / _ \/ _ \/ __| '_ \ 
# | | | | (_| | | | |  __/  __/\__ \ | | |
# \_| |_/\__,_|_| |_|\___|\___||___/_| |_|
# Date:   2020-03-05 18:57:43
# Last Modified time: 2020-03-24 02:08:27

import numpy as np
from scipy.spatial import cKDTree 

class Node():
	def __init__(self,x,y,cost=0,ind=None):
		self.x=x
		self.y=y
		self.cost=cost
		self.ind=ind

def motion_model():
	motion = [[1,0,1],
			[0,1,1],
			[-1,0,1],
			[0,-1,1],
			[1,1,np.sqrt(2)],
			[-1,-1,np.sqrt(2)],
			[1,-1,np.sqrt(2)],
			[-1,1,np.sqrt(2)]]
	return motion


class KDTree():
	def __init__(self,data):
		self.tree=cKDTree(data)

	def search_tree(self,inp,k=1):
		
		if len(inp.shape)>=2:
			index=[]
			dist=[]
			for i in inp.T:
				q_dist,q_index=self.tree.query(i,k=k)
				dist.append(q_dist)
				index.append(q_index)
			return dist,index
		
		dist,index=self.tree.query(inp,k=k)
		return dist,index

	def search_distance(self,input,r):
		index=self.tree.query_ball_point(inp,r)
		return index