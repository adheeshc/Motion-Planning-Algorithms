# -*- coding: utf-8 -*-
#   ___      _ _                    _     
#  / _ \    | | |                  | |    
# / /_\ \ __| | |__   ___  ___  ___| |__  
# |  _  |/ _` | '_ \ / _ \/ _ \/ __| '_ \ 
# | | | | (_| | | | |  __/  __/\__ \ | | |
# \_| |_/\__,_|_| |_|\___|\___||___/_| |_|
# Date:   2020-03-24 02:44:07
# Last Modified time: 2020-03-24 03:22:46

from prm import PRM

if __name__=="__main__":

	grid=[60,60]
	start_x=5
	start_y=5
	goal_x=55
	goal_y=55
	resolution=1
	robot_size=1

	prm=PRM(grid, start_x, start_y, goal_x, goal_y)
	prm.plot(record=True)
