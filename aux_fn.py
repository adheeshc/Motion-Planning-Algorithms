# -*- coding: utf-8 -*-
#   ___      _ _                    _     
#  / _ \    | | |                  | |    
# / /_\ \ __| | |__   ___  ___  ___| |__  
# |  _  |/ _` | '_ \ / _ \/ _ \/ __| '_ \ 
# | | | | (_| | | | |  __/  __/\__ \ | | |
# \_| |_/\__,_|_| |_|\___|\___||___/_| |_|
# Date:   2020-03-05 18:57:43
# Last Modified time: 2020-03-06 12:31:06

import numpy as np

class Node():
	def __init__(self,x,y,cost,ind):
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
