### Author: John Grezmak
###
### ---Demo for using the linear_foot_path_solver function to produce a pull
### inward motion. Results are stored as arrays of PWM pulse that can be sent 
### as commands to the Maestro servo controller.

import pickle

import numpy as np
import time

from gait_defs import convert_to_PWM, create_pull_inward
from med_Sebastian_info import *

# ---Define list of leg identities for simplifying following code
identities = ['R1', 'L2', 'R3', 'L1', 'R2', 'L3']

# ---Define dictionary of tripod gait parameters
gait_params = {'init_conds_R1': [45, -55, -55],		# Initial conditions for leg R1 (units: degrees)
				'init_conds_L2': [0.1, -55, -55],	# Initial conditions for leg L2 (units: degrees)
				'init_conds_R3': [-45, -55, -55],	# Initial conditions for leg R3 (units: degrees)
				'vx': 31.513693/np.sqrt(2)/4,					# Foot tip velocity in x-direction during stance/swing
				'vy': 31.513693/np.sqrt(2)/4,					# Foot tip velocity in y-direction during stance/swing
				'vh': 20,
				'points': 10001,					# Number of points to compute in theta relation solutions
				'dt': 0.0001}						# Time step for computing theta relation solutions
				
# ---Compute theta relations for full pull inward motion
thetas = create_pull_inward(gait_params, robot_dims)

# Convert theta relations to PWM signal pulse widths for Maestro controller
step = 1000 # Compute PWM command every 'step' time steps of computed theta relations
prec = 1 # Round PWM commands to nearest 'prec' value

thetas_PWM = convert_to_PWM(thetas, step, prec, gait_params['points'], gait_params['dt'],
							center_pos_info, robot_dims)
							
# Define reset position info (assuming right tripod starts in stance)
reset_pos_info = dict()
for identity in identities:
	if identity in identities:
		reset_pos_info[identity] = [] + [thetas[0]['theta' + str(k + 1) + '_' + identity][0] for k in range(3)]



# Store PWM signal pulse widths relations to a file
filename = r'/home/pi/Documents/John/Data/gait_parameters/pull_inward_1.sav'
f = open(filename, 'w')
f.close()
with open(filename, 'wb') as f:
	pickle.dump(thetas_PWM, f)
	pickle.dump(reset_pos_info, f)


print('PWM signal pulse width relations for a pull inward motion saved to ' + filename)
