# -*- coding: utf-8 -*-
#   ___      _ _                    _     
#  / _ \    | | |                  | |    
# / /_\ \ __| | |__   ___  ___  ___| |__  
# |  _  |/ _` | '_ \ / _ \/ _ \/ __| '_ \ 
# | | | | (_| | | | |  __/  __/\__ \ | | |
# \_| |_/\__,_|_| |_|\___|\___||___/_| |_|
# Date:   2020-03-05 16:27:57
# Last Modified time: 2020-03-05 17:09:14

from init import *
import matplotlib.pyplot as plt

if __name__=="__main__":
	grid=[300,150]
	#grid=[150,300]
	#grid=[300,300]
	res=1
	rob_size=1
	env=Environment(grid,res,rob_size)
	env.show_map()

