### Author: John Grezmak
###
### ---Demo for using the linear_foot_path_solver function to produce a forward
### moving symmetric tripod gait. Results are stored as arrays of PWM pulse
### widths that can be sent as commands to the Maestro servo controller.

import pickle

import numpy as np
import time

from gait_defs import convert_to_PWM, create_tripod_gait
from med_Sebastian_info import *

# ---Define list of leg identities for simplifying following code
identities = ['R1', 'L2', 'R3', 'L1', 'R2', 'L3']

# ---Define dictionary of tripod gait parameters
gait_params = {'init_conds_R1': [55, -55, -55],		# Initial conditions for leg R1 (units: degrees)
				'init_conds_L2': [-10, -55, -55],	# Initial conditions for leg L2 (units: degrees)
				'init_conds_R3': [-40, -55, -55],	# Initial conditions for leg R3 (units: degrees)
				#'vx': 0,							# Foot tip velocity in x-direction during stance/swing
				'vx': 31.513693/2,
				#'vy': 31.513693,					# Foot tip velocity in y-direction during stance/swing
				'vy': 27.291659,
				'vh': 25,		   					# Foot tip velocity in z-direction during swing/plant
				'points': 10001,					# Number of points to compute in theta relation solutions
				'dt': 0.0001,						# Time step for computing theta relation solutions
				'n_phases': 6}						# Number of phases in tripod gait (4 or 6)
				
# ---Compute theta relations for full tripod gait
thetas = create_tripod_gait(gait_params, robot_dims)

# Convert theta relations to PWM signal pulse widths for Maestro controller
step = 1000 # Compute PWM command every 'step' time steps of computed theta relations
prec = 1 # Round PWM commands to nearest 'prec' value

thetas_PWM = convert_to_PWM(thetas, step, prec, gait_params['points'], gait_params['dt'],
							center_pos_info, robot_dims)

# Define reset position info (assuming right tripod starts in stance)
reset_pos_info = dict()
for identity in identities:
	if identity in ['R1', 'L2', 'R3']:
		reset_pos_info[identity] = [] + [thetas_PWM[0]['theta' + str(k + 1) + '_' + identity][0] for k in range(3)]
			
	elif identity in ['L1', 'R2', 'L3']:
		if gait_params['n_phases'] == 4:
			reset_pos_info[identity] = [] + [thetas_PWM[1]['theta' + str(k + 1) + '_' + identity][0] for k in range(3)]
		elif gait_params['n_phases'] == 6:
			print('check 5')
			reset_pos_info[identity] = [] + [thetas_PWM[1]['theta' + str(k + 1) + '_' + identity][0] for k in range(3)]
			
			
			

print(thetas_PWM[1])
print(reset_pos_info)
# Store PWM signal pulse widths relations to a file
#filename = r'/home/pi/Documents/John/Data/gait_parameters/tripod_gait_4phase_1.sav'
#filename = r'/home/pi/Documents/John/Data/gait_parameters/tripod_gait_6phase_1.sav'
filename = r'/home/pi/Documents/John/Data/gait_parameters/inward_tripod_gait_6phase_1.sav'
f = open(filename, 'w')
f.close()
with open(filename, 'wb') as f:
	pickle.dump(thetas_PWM, f)
	pickle.dump(reset_pos_info, f)


print('PWM signal pulse width relations for a ' + str(gait_params['n_phases']) + ' phase tripod gait saved to ' + filename)
