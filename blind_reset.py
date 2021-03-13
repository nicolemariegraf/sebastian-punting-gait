### Author: John Grezmak
###
### Reset the Sebastian robot using reset function with mode=1

import time

from med_Sebastian_info import *
from Sebastian_library_Maestro import Sebastian

# Create instance of Sebastian class
reset_config = reset3_pos_info 		# Choice of reset configuration
robot = Sebastian(center_pos_info, pins_info, reset_config, robot_dims)
error = robot.mu.get_error()
if error:
	print(error)
else:
	print('Robot initialized')
	

# Reset robot with mode=1 (blind reset)
robot.reset(mode=1)
