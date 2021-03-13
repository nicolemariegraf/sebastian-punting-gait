### Author: John Grezmak
### This file allows the 18-dof Sebastian robot to perform the forward_gait_V1 gait

import time

from gait_defs import blind_grip
from med_Sebastian_info import *
from Sebastian_library_Maestro import Sebastian

### Create instance of Sebastian class, check for errors
reset_config = reset4_pos_info
robot = Sebastian(center_pos_info, pins_info, reset_config)
error = robot.mu.get_error()
if error:
	print(error)
else:
	print('Robot initialized...')


### Reset robot according to reset_config
print('Resetting servos to reset positions...')
robot.reset(mode=2)
time.sleep(1)

### Define movement of ankle servos for blind grip
a_mov = 200
sensing = False
tripod = "both"

### Start blind grip
print('Performing blind grip of size ' + str(int(a_mov/10)) + ' degrees...')
blind_grip(robot, a_mov, sensing, tripod)

# Close all serial communication
error = robot.mu.get_error()
if error:
	print(error)

robot.mu.close()
