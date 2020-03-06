# -*- coding: utf-8 -*-
#   ___      _ _                    _     
#  / _ \    | | |                  | |    
# / /_\ \ __| | |__   ___  ___  ___| |__  
# |  _  |/ _` | '_ \ / _ \/ _ \/ __| '_ \ 
# | | | | (_| | | | |  __/  __/\__ \ | | |
# \_| |_/\__,_|_| |_|\___|\___||___/_| |_|
# Date:   2020-03-05 16:27:57
# Last Modified time: 2020-03-06 01:57:19

from env import *

if __name__=="__main__":
	#grid=[30,15]
	#grid=[15,30]
	grid=[60,60]
	res=1
	rob_size=1
	env=Environment(grid,res,rob_size)
	env.show_map()