### Author: John Grezmak
###
### ---Demo for implementing tripod or inward tripod gait on Sebastian robot

import pickle
import time

from gait_defs import pull_inward
from med_Sebastian_info import *
from Sebastian_library_Maestro import Sebastian

# Load PWM signal relations from file
filename = r'/home/pi/Documents/John/Data/gait_parameters/pull_inward_1.sav'
with open(filename, 'rb') as f:
	thetas_PWM = pickle.load(f)
	reset_config = pickle.load(f)
	

### Create instance of Sebastian class, check for errors
robot = Sebastian(center_pos_info, pins_info, reset_config, robot_dims)
error = robot.mu.get_error()
if error:
	print(error)
else:
	print('Robot initialized...')


### Reset robot according to reset_config
print('Resetting servos to reset positions...')
robot.reset(mode=2)
time.sleep(2)

### Start tripod gait
n_steps = 1
pull_inward(robot, thetas_PWM, n_steps)

# Close serial communication
error = robot.mu.get_error()
if error:
	print(error)

robot.mu.close()
