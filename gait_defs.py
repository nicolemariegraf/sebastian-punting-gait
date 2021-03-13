### Author: John Grezmak
### Python library of gait-related functions for the 18-dof Sebastian robot

import itertools
import math
import pickle

import matplotlib.pyplot as plt
import numpy as np
import time

def linear_foot_path_solver(robot_dims, thetas0, vx, vy, vh, points, dt):
	# This function computes leg angle vs. time relations resulting in linear
	# footpaths for a 3-DOF leg, given initial leg angles and desired foot path 
	# velocities. 
	#
	# robot_dims:		Robot leg dimensions (Python dictionary)
	# thetas0:			Initial leg angles of leg (Python list, units = deg, format: [hip, knee, ankle])
	# vx:				Lateral speed of foot tip path (float, units = mm/s)
	# vy:				Forward speed of foot tip path (float, units = mm/s)
	# vh:				Vertical speed of foot tip path (float, units = mm/s)
	# points:			Number of time points to solve foot path equations (int)
	# dt:				Step size for solving foot path equations (float, units = s)
	
	# Define leg dimensions
	L0 = robot_dims['L0']	# L0 dimension
	L1 = robot_dims['L1']	# L1 dimension
	L2 = robot_dims['L2']	# L2 dimension
	
	# Define constant for converting degrees to radians
	T = np.pi/180
	
	# Define initial conditions
	theta1_0 = thetas0[0] # Hip angle (ϴ1)
	theta2_0 = thetas0[1] # Knee angle (ϴ2)
	theta3_0 = thetas0[2] # Ankle angle (ϴ3)
	
	# Initialize vectors to store values of time, theta1, theta2, theta3 and derivatives
	times = np.linspace(0, dt*points, points) # Time
	theta1_vec = np.zeros([points, ])         # Theta1
	theta2_vec = np.zeros([points, ])         # Theta2
	theta3_vec = np.zeros([points, ])         # Theta3
	dtheta1_vec = np.zeros([points, ])        # dTheta1_dt
	dtheta2_vec = np.zeros([points, ])        # dTheta2_dt
	dtheta3_vec = np.zeros([points, ])        # dTheta3_dt
	
	# Initialize theta1, theta2, theta3 and derivatives
	theta1 = theta1_0*T
	theta1_vec[0] = theta1
	theta2 = theta2_0*T
	theta2_vec[0] = theta2
	theta3 = theta3_0*T
	theta3_vec[0] = theta3

	dtheta1 = 1*15*T
	dtheta1_vec[0] = dtheta1
	dtheta2 = 1*1*T
	dtheta2_vec[0] = dtheta2
	dtheta3 = -1*1*T
	dtheta3_vec[0] = dtheta3
	
	for k in range(1, points):
		# Update theta1 according to Euler method
		dtheta1_dt = (-1*vx + np.cos(theta1_vec[k - 1])*(-L1*np.sin(theta2_vec[k - 1])*dtheta2_vec[k - 1] - \
					L2*np.sin(theta2_vec[k - 1] + theta3_vec[k - 1])*(dtheta2_vec[k - 1] + \
					dtheta3_vec[k - 1])))/(np.sin(theta1_vec[k - 1])*(L0 + L1*np.cos(theta2_vec[k - 1]) + \
					L2*np.cos(theta2_vec[k - 1] + theta3_vec[k - 1])))
		theta1 = theta1 + dt*dtheta1_dt
		theta1_vec[k] = theta1
		dtheta1_vec[k] = dtheta1_dt
		
		# Update theta2 according to Euler method
		dtheta2_dt = ((vy + vx*(1/np.tan(theta1_vec[k - 1])))*np.sin(theta1_vec[k - 1]) + \
					  L2*np.sin(theta2_vec[k - 1] + theta3_vec[k - 1])*dtheta3_vec[k - 1])/\
					(-L1*np.sin(theta2_vec[k - 1]) - L2*np.sin(theta2_vec[k - 1] + theta3_vec[k - 1]))
		theta2 = theta2 + dt*dtheta2_dt
		theta2_vec[k] = theta2
		dtheta2_vec[k] = dtheta2_dt
		
		# Update theta3 according to Euler method
		dtheta3_dt = (vh - dtheta2_vec[k - 1]*(L1*np.cos(theta2_vec[k - 1]) + L2*np.cos(theta2_vec[k - 1] + \
					theta3_vec[k - 1])))/(L2*np.cos(theta2_vec[k - 1] + theta3_vec[k - 1]))
		theta3 = theta3 + dt*dtheta3_dt
		theta3_vec[k] = theta3
		dtheta3_vec[k] = dtheta3_dt
		
	
	# Convert thetas back to degrees
	theta1_vec = theta1_vec/T
	theta2_vec = theta2_vec/T
	theta3_vec = theta3_vec/T
	
	return theta1_vec, theta2_vec, theta3_vec
	
def thetas_plotter(theta_vecs):
	# This functions takes a dictionary of theta values for all legs of
	# the robot and plots the theta vs. time relations.
	#
	# theta_vecs:	Python dictionary of theta values for all legs
	
	# Define time vector
	points = theta_vecs['points']
	dt = theta_vecs['dt']
	times = np.linspace(0, dt*points, points)
	
	# Plot theta vs. time relations
	fig, ((ax1, ax2, ax3), (ax4, ax5, ax6), (ax7, ax8, ax9)) = plt.subplots(3, 3, figsize=(20, 20))
	plt.subplots_adjust(hspace=0.3)
	
	keys = ['theta1_R1', 'theta2_R1', 'theta3_R1', 'theta1_L2', 'theta2_L2', 'theta3_L2', \
				'theta1_R3', 'theta2_R3', 'theta3_R3']
	colors = ['blue', 'darkorange', 'green']
				
	for m in range(3):
		eval('ax'+str(3*m + 1)).plot(times, theta_vecs[keys[3*m]], linewidth=2.5, color=colors[m])
		eval('ax'+str(3*m + 1)).set_ylabel('Angle (degrees)', fontsize=14)
		eval('ax'+str(3*m + 1)).set_title(keys[3*m], fontsize=16)
		eval('ax'+str(3*m + 1)).tick_params(axis='x', labelsize=13)
		eval('ax'+str(3*m + 1)).tick_params(axis='y', labelsize=13)
		eval('ax'+str(3*m + 1)).set_xlim([0, dt*points])
		eval('ax'+str(3*m + 1)).set_ylim([np.min(theta_vecs[keys[3*m]][20:]) - 1, np.max(theta_vecs[keys[3*m]][20:]) + 1])
		
		eval('ax'+str(3*m + 2)).plot(times, theta_vecs[keys[3*m + 1]], linewidth=2.5, color=colors[m])
		eval('ax'+str(3*m + 2)).set_title(keys[3*m + 1], fontsize=16)
		eval('ax'+str(3*m + 2)).tick_params(axis='x', labelsize=13)
		eval('ax'+str(3*m + 2)).tick_params(axis='y', labelsize=13)
		eval('ax'+str(3*m + 2)).set_xlim([0, dt*points])
		eval('ax'+str(3*m + 2)).set_ylim([np.min(theta_vecs[keys[3*m + 1]][20:]) - 1, np.max(theta_vecs[keys[3*m + 1]][20:]) + 1])
		
		eval('ax'+str(3*m + 3)).plot(times, theta_vecs[keys[3*m + 2]], linewidth=2.5, color=colors[m])
		eval('ax'+str(3*m + 3)).set_title(keys[3*m + 2], fontsize=16)
		eval('ax'+str(3*m + 3)).tick_params(axis='x', labelsize=13)
		eval('ax'+str(3*m + 3)).tick_params(axis='y', labelsize=13)
		eval('ax'+str(3*m + 3)).set_xlim([0, dt*points])
		eval('ax'+str(3*m + 3)).set_ylim([np.min(theta_vecs[keys[3*m + 2]][20:]) - 1, np.max(theta_vecs[keys[3*m + 2]][20:]) + 1])
		
		if m == 2:
			eval('ax'+str(3*m + 1)).set_xlabel('Time (s)', fontsize=14)
			eval('ax'+str(3*m + 2)).set_xlabel('Time (s)', fontsize=14)
			eval('ax'+str(3*m + 3)).set_xlabel('Time (s)', fontsize=14)
			
		
		
	plt.show()
	
def convert_to_PWM(thetas, step, prec, points, dt, center_pos_info, robot_dims):
	### Converts theta relations in units of degrees to PWM signals for servo control
	#
	# thetas:			Python list of theta relations
	# step:				Compute PWM command every 'step' time steps of computed theta relations (int)
	# prec:				Round PWM commands to nearest 'prec' value (int)
	# points:
	# dt:
	# center_pos_info:	Python dictionary for servo center position info
	# robot_dims:		Python dictionary for robot dimension info
	#
	# returns:
	
	# ---Unpack values from "thetas"
	if len(thetas) == 1:
		gait_type = 'pull_inward'
	elif len(thetas) == 3:
		print('check')
		thetas_stance = thetas[0]
		thetas_swing = thetas[1]
		thetas_plant = thetas[2]
		gait_type = 'tripod'
	elif len(thetas) == 4:
		print('check 4')
		thetas_stance = thetas[0]
		thetas_raise = thetas[1]
		thetas_swing = thetas[2]
		thetas_plant = thetas[3]
		gait_type = 'tripod'
		
	
	# ---Define list of leg identities for simplifying following code
	identities = ['R1', 'L2', 'R3', 'L1', 'R2', 'L3']
	
	if gait_type == 'pull_inward':
		# Initialize dictionary to store PWM relations
		thetas_PWM = dict()
		
		# Convert thetas to PWM
		for identity in identities:
			for k in range(3):
				thetas_PWM['theta' + str(k + 1) + '_' + identity] = np.zeros([len(thetas[0]['theta1_R1'][0::step]), ], dtype=int)
				for m, n in enumerate(thetas[0]['theta' + str(k + 1) + '_' + identity][0:thetas[0]['points']:step]):
					if k != 2:
						thetas_PWM['theta' + str(k + 1) + '_' + identity][m] = int(int(math.ceil((center_pos_info[identity][k] + 10*n) / prec))*prec)
					else:
						thetas_PWM['theta' + str(k + 1) + '_' + identity][m] = int(int(math.ceil((center_pos_info[identity][k] - 10*(n + robot_dims['offset_angle'])) / prec))*prec)
						
						
						
		
		# Store additional information about parameters used to compute relations
		thetas_PWM['points'] = thetas[0]['points']
		thetas_PWM['dt'] = thetas[0]['dt']
		thetas_PWM['step'] = step			
	elif gait_type == 'tripod':
		# Initialize dictionaries to store PWM relations
		print('check')
		thetas_stance_PWM = dict()
		thetas_swing_PWM = dict()
		thetas_plant_PWM = dict()
		if len(thetas) == 4:
			thetas_raise_PWM = dict()
			
		
		# Convert thetas to PWM for each gait phase
		for identity in identities:
			for k in range(3):
				thetas_stance_PWM['theta' + str(k + 1) + '_' + identity] = np.zeros([len(thetas_stance['theta1_R1'][0::step]), ], dtype=int)
				for m, n in enumerate(thetas_stance['theta' + str(k + 1) + '_' + identity][0:thetas_swing['points']:step]):
					if k != 2:
						thetas_stance_PWM['theta' + str(k + 1) + '_' + identity][m] = int(int(math.ceil((center_pos_info[identity][k] + 10*n) / prec))*prec)
					else:
						thetas_stance_PWM['theta' + str(k + 1) + '_' + identity][m] = int(int(math.ceil((center_pos_info[identity][k] - 10*(n + robot_dims['offset_angle'])) / prec))*prec)
						
					
				thetas_swing_PWM['theta' + str(k + 1) + '_' + identity] = np.zeros([len(thetas_swing['theta1_R1'][0::step]), ], dtype=int)
				for m, n in enumerate(thetas_swing['theta' + str(k + 1) + '_' + identity][0:thetas_swing['points']:step]):
					if k != 2:
						thetas_swing_PWM['theta' + str(k + 1) + '_' + identity][m] = int(int(math.ceil((center_pos_info[identity][k] + 10*n) / prec))*prec)
					else:
						thetas_swing_PWM['theta' + str(k + 1) + '_' + identity][m] = int(int(math.ceil((center_pos_info[identity][k] - 10*(n + robot_dims['offset_angle'])) / prec))*prec)
						
					
				thetas_plant_PWM['theta' + str(k + 1) + '_' + identity] = np.zeros([len(thetas_plant['theta1_R1'][0::step]), ], dtype=int)
				for m, n in enumerate(thetas_plant['theta' + str(k + 1) + '_' + identity][0:thetas_swing['points']:step]):
					if k != 2:
						thetas_plant_PWM['theta' + str(k + 1) + '_' + identity][m] = int(int(math.ceil((center_pos_info[identity][k] + 10*n) / prec))*prec)
					else:
						thetas_plant_PWM['theta' + str(k + 1) + '_' + identity][m] = int(int(math.ceil((center_pos_info[identity][k] - 10*(n + robot_dims['offset_angle'])) / prec))*prec)
						
						
				if len(thetas) == 4:
					print('check 6')
					thetas_raise_PWM['theta' + str(k + 1) + '_' + identity] = np.zeros([len(thetas_raise['theta1_R1'][0::step]), ], dtype=int)
					for m, n in enumerate(thetas_raise['theta' + str(k + 1) + '_' + identity][0:thetas_swing['points']:step]):
						if k != 2:
							thetas_raise_PWM['theta' + str(k + 1) + '_' + identity][m] = int(int(math.ceil((center_pos_info[identity][k] + 10*n) / prec))*prec)
						else:
							thetas_raise_PWM['theta' + str(k + 1) + '_' + identity][m] = int(int(math.ceil((center_pos_info[identity][k] - 10*(n + robot_dims['offset_angle'])) / prec))*prec)
							
						




		# Store additional information about parameters used to compute relations
		thetas_stance_PWM['points'] = thetas_stance['points']
		thetas_stance_PWM['dt'] = thetas_stance['dt']
		thetas_stance_PWM['step'] = step
		thetas_swing_PWM['points'] = thetas_swing['points']
		thetas_swing_PWM['dt'] = thetas_swing['dt']
		thetas_swing_PWM['step'] = step
		thetas_plant_PWM['points'] = thetas_plant['points']
		thetas_plant_PWM['dt'] = thetas_plant['dt']
		thetas_plant_PWM['step'] = step
		if len(thetas) == 4:
			thetas_raise_PWM['points'] = thetas_raise['points']
			thetas_raise_PWM['dt'] = thetas_raise['dt']
			thetas_raise_PWM['step'] = step
			
		
	
	if len(thetas) == 3: # 4-phase
		thetas_PWM = [thetas_stance_PWM, thetas_swing_PWM, thetas_plant_PWM]
	elif len(thetas) == 4: # 6-phase
		thetas_PWM = [thetas_stance_PWM, thetas_raise_PWM, thetas_swing_PWM, thetas_plant_PWM]
	
	return thetas_PWM
	
def create_tripod_gait(gait_params, robot_dims):
	### Computes theta relations in degrees based on desired tripod gait 
	### parameters and stores results in Python dictionaries.
	#
	# gait_params:	Python dictionary of tripod gait parameters
	# robot_dims:	Python dictionary for robot dimension info
	#
	# returns: thetas (list)
	
	# Unpack parameter values from gait_params
	vx = gait_params['vx']
	vy = gait_params['vy']
	vh = gait_params['vh']
	points = gait_params['points']
	dt = gait_params['dt']
	n_phases = gait_params['n_phases']
	
	# ---Define list of leg identities for simplifying following code
	identities = ['R1', 'L2', 'R3', 'L1', 'R2', 'L3']
	
	# ---Compute theta relations during stance movement
	thetas_stance = dict()

	# Define foot path paratmeters for each leg of tripod (assume right tripod)
	# Format: [robot_dims, [theta1, theta2, theta3], vx, vy, vh, points, dt]
	params_stance_rf = [robot_dims, gait_params['init_conds_R1'], -1*vx, -1*vy, 0, points, dt]	# Front tripod leg (R1)
	params_stance_lm = [robot_dims, gait_params['init_conds_L2'], -1*vx, vy, 0, points, dt]	# Middle tripod leg (L2)
	params_stance_rb = [robot_dims, gait_params['init_conds_R3'], -1*vx, -1*vy, 0, points, dt] # Back tripod leg (R3)
	params_stance = [params_stance_rf, params_stance_lm, params_stance_rb]

	# Compute thetas for right tripod using linear_foot_path_solver() and assign
	# values for left tripod based on symmetric tripod gait
	for k in range(3):
		thetas_stance['theta1_' + identities[k]], thetas_stance['theta2_' + identities[k]], \
			thetas_stance['theta3_' + identities[k]] = linear_foot_path_solver(*params_stance[k])
			
		thetas_stance['theta1_' + identities[k + 3]] = -1*thetas_stance['theta1_' + identities[k]]
		thetas_stance['theta2_' + identities[k + 3]] = thetas_stance['theta2_' + identities[k]]
		thetas_stance['theta3_' + identities[k + 3]] = thetas_stance['theta3_' + identities[k]]
		
	thetas_stance['points'] = points
	thetas_stance['dt'] = dt

	# Plot resulitng theta vs. time relations for stance phase
	thetas_plotter(thetas_stance)
	
	# ---Compute theta relations during swing movement (4-phase) or raising
	# movement (6-phase)
	# Define foot path paratmeters for each leg of tripod
	if n_phases == 4: # 4-phase
		thetas_swing = dict()
		params_swing_rf = [robot_dims, [thetas_stance['theta1_R1'][-1], thetas_stance['theta2_R1'][-1], \
						thetas_stance['theta3_R1'][-1]], vx, vy, vh, points, dt]	# Front tripod leg (rf)
		params_swing_lm = [robot_dims, [thetas_stance['theta1_L2'][-1], thetas_stance['theta2_L2'][-1], \
						thetas_stance['theta3_L2'][-1]], vx, -1*vy, vh, points, dt]	# Middle tripod leg (lm)
		params_swing_rb = [robot_dims, [thetas_stance['theta1_R3'][-1], thetas_stance['theta2_R3'][-1], \
						thetas_stance['theta3_R3'][-1]], vx, vy, vh, points, dt] # Back tripod leg (rb)
						
		params_swing = [params_swing_rf, params_swing_lm, params_swing_rb]
	elif n_phases == 6:
		thetas_raise = dict()
		params_raise_rf = [robot_dims, [thetas_stance['theta1_R1'][-1], thetas_stance['theta2_R1'][-1], \
						thetas_stance['theta3_R1'][-1]], 0, 0, vh, points, dt]	# Front tripod leg (rf)
		params_raise_lm = [robot_dims, [thetas_stance['theta1_L2'][-1], thetas_stance['theta2_L2'][-1], \
						thetas_stance['theta3_L2'][-1]], 0, 0, vh, points, dt]	# Middle tripod leg (lm)
		params_raise_rb = [robot_dims, [thetas_stance['theta1_R3'][-1], thetas_stance['theta2_R3'][-1], \
						thetas_stance['theta3_R3'][-1]], 0, 0, vh, points, dt] # Back tripod leg (rb)
		
		params_raise = [params_raise_rf, params_raise_lm, params_raise_rb]
		

	# Compute thetas for right tripod using linear_foot_path_solver() and assign
	# values for left tripod based on symmetric tripod gait
	if n_phases == 4:
		for k in range(3): # 6-phase
			thetas_swing['theta1_' + identities[k]], thetas_swing['theta2_' + identities[k]], \
				thetas_swing['theta3_' + identities[k]] = linear_foot_path_solver(*params_swing[k])
				
			thetas_swing['theta1_' + identities[k + 3]] = -1*thetas_swing['theta1_' + identities[k]]
			thetas_swing['theta2_' + identities[k + 3]] = thetas_swing['theta2_' + identities[k]]
			thetas_swing['theta3_' + identities[k + 3]] = thetas_swing['theta3_' + identities[k]]
			
		thetas_swing['points'] = points
		thetas_swing['dt'] = dt
		
		# Plot resulitng theta vs. time relations
		thetas_plotter(thetas_swing)
	elif n_phases == 6:
		for k in range(3): # 6-phase
			thetas_raise['theta1_' + identities[k]], thetas_raise['theta2_' + identities[k]], \
				thetas_raise['theta3_' + identities[k]] = linear_foot_path_solver(*params_raise[k])
				
			thetas_raise['theta1_' + identities[k + 3]] = -1*thetas_raise['theta1_' + identities[k]]
			thetas_raise['theta2_' + identities[k + 3]] = thetas_raise['theta2_' + identities[k]]
			thetas_raise['theta3_' + identities[k + 3]] = thetas_raise['theta3_' + identities[k]]
			
		thetas_raise['points'] = points
		thetas_raise['dt'] = dt
		
		# Plot resulitng theta vs. time relations
		thetas_plotter(thetas_raise)
		
	
	# ---Compute theta relations during plant movement (4-phase) or swing
	# movement (6-phase)
	# Define foot path paratmeters for each leg of tripod
	if n_phases == 4: # 4-phase
		thetas_plant = dict()
		params_plant_rf = [robot_dims, [thetas_swing['theta1_R1'][-1], thetas_swing['theta2_R1'][-1], \
						thetas_swing['theta3_R1'][-1]], 0, 0, -1*vh, points, dt]	# Front tripod leg (rf)
		params_plant_lm = [robot_dims, [thetas_swing['theta1_L2'][-1], thetas_swing['theta2_L2'][-1], \
						thetas_swing['theta3_L2'][-1]], 0, 0, -1*vh, points, dt]	# Middle tripod leg (lm)
		params_plant_rb = [robot_dims, [thetas_swing['theta1_R3'][-1], thetas_swing['theta2_R3'][-1], \
						thetas_swing['theta3_R3'][-1]], 0, 0, -1*vh, points, dt] # Back tripod leg (rb)
		params_plant = [params_plant_rf, params_plant_lm, params_plant_rb]
	elif n_phases == 6: # 6-phase
		thetas_swing = dict()
		params_swing_rf = [robot_dims, [thetas_raise['theta1_R1'][-1], thetas_raise['theta2_R1'][-1], \
						thetas_raise['theta3_R1'][-1]], vx, vy, 0, points, dt]	# Front tripod leg (rf)
		params_swing_lm = [robot_dims, [thetas_raise['theta1_L2'][-1], thetas_raise['theta2_L2'][-1], \
						thetas_raise['theta3_L2'][-1]], vx, -1*vy, 0, points, dt]	# Middle tripod leg (lm)
		params_swing_rb = [robot_dims, [thetas_raise['theta1_R3'][-1], thetas_raise['theta2_R3'][-1], \
						thetas_raise['theta3_R3'][-1]], vx, vy, 0, points, dt] # Back tripod leg (rb)
		params_swing = [params_swing_rf, params_swing_lm, params_swing_rb]
		

	# Compute thetas for right tripod using linear_foot_path_solver() and assign
	# values for left tripod based on symmetric tripod gait
	if n_phases == 4: # 4-phase
		for k in range(3):
			thetas_plant['theta1_' + identities[k]], thetas_plant['theta2_' + identities[k]], \
				thetas_plant['theta3_' + identities[k]] = linear_foot_path_solver(*params_plant[k])
				
			thetas_plant['theta1_' + identities[k + 3]] = -1*thetas_plant['theta1_' + identities[k]]
			thetas_plant['theta2_' + identities[k + 3]] = thetas_plant['theta2_' + identities[k]]
			thetas_plant['theta3_' + identities[k + 3]] = thetas_plant['theta3_' + identities[k]]
			
		thetas_plant['points'] = points
		thetas_plant['dt'] = dt

		# Plot resulitng theta vs. time relations
		thetas_plotter(thetas_plant)
	elif n_phases == 6: # 6-phase
		for k in range(3):
			thetas_swing['theta1_' + identities[k]], thetas_swing['theta2_' + identities[k]], \
				thetas_swing['theta3_' + identities[k]] = linear_foot_path_solver(*params_swing[k])
				
			thetas_swing['theta1_' + identities[k + 3]] = -1*thetas_swing['theta1_' + identities[k]]
			thetas_swing['theta2_' + identities[k + 3]] = thetas_swing['theta2_' + identities[k]]
			thetas_swing['theta3_' + identities[k + 3]] = thetas_swing['theta3_' + identities[k]]
			
		thetas_swing['points'] = points
		thetas_swing['dt'] = dt

		# Plot resulitng theta vs. time relations
		thetas_plotter(thetas_swing)
		
	
	# ---Compute theta relations during plant movement (6-phase)
	# Define foot path paratmeters for each leg of tripod
	if n_phases == 6: # 6-phase
		thetas_plant = dict()
		params_plant_rf = [robot_dims, [thetas_swing['theta1_R1'][-1], thetas_swing['theta2_R1'][-1], \
						thetas_swing['theta3_R1'][-1]], 0, 0, -1*vh, points, dt]	# Front tripod leg (rf)
		params_plant_lm = [robot_dims, [thetas_swing['theta1_L2'][-1], thetas_swing['theta2_L2'][-1], \
						thetas_swing['theta3_L2'][-1]], 0, 0, -1*vh, points, dt]	# Middle tripod leg (lm)
		params_plant_rb = [robot_dims, [thetas_swing['theta1_R3'][-1], thetas_swing['theta2_R3'][-1], \
						thetas_swing['theta3_R3'][-1]], 0, 0, -1*vh, points, dt] # Back tripod leg (rb)
		params_plant = [params_plant_rf, params_plant_lm, params_plant_rb]
		
		# Compute thetas for right tripod using linear_foot_path_solver() and assign
		# values for left tripod based on symmetric tripod gait
		for k in range(3):
			thetas_plant['theta1_' + identities[k]], thetas_plant['theta2_' + identities[k]], \
				thetas_plant['theta3_' + identities[k]] = linear_foot_path_solver(*params_plant[k])
				
			thetas_plant['theta1_' + identities[k + 3]] = -1*thetas_plant['theta1_' + identities[k]]
			thetas_plant['theta2_' + identities[k + 3]] = thetas_plant['theta2_' + identities[k]]
			thetas_plant['theta3_' + identities[k + 3]] = thetas_plant['theta3_' + identities[k]]
			
		thetas_plant['points'] = points
		thetas_plant['dt'] = dt

		# Plot resulitng theta vs. time relations
		thetas_plotter(thetas_plant)
		
	
	if n_phases == 4:
		thetas = [thetas_stance, thetas_swing, thetas_plant]
	elif n_phases == 6:
		thetas = [thetas_stance, thetas_raise, thetas_swing, thetas_plant]
		
	return thetas
	
def tripod_gait(robot, PWM, n_steps, multiplier=1/6, direction='Forward', init_only=False):
	# This function implements the tripod gait given computed values of
	# theta vs. time relations for this gait.
	#
	# robot:		Instance of Sebastian class
	# PWM:			Python list of PWM relations for tripod gait [stance, swing, plant]
	# n_steps:		Number of steps of tripod gait
	# multiplier:	Parameter for determining time between successive target serial commands, default is 1/6
	# direction:	Direction of motion resulting from tripod gait relative to front (str, default = 'Forward', alternative is 'Reverse')
	# init_only:	If True, initializes gait from reset position
	
	# --- Initialize PWM vs. time relations
	if len(PWM) == 3:
		n_phases = 4 # 4-phase tripod gait
		
		thetas_stance_PWM = PWM[0]
		thetas_swing_PWM = PWM[1]
		thetas_plant_PWM = PWM[2]
	elif len(PWM) == 4:
		n_phases = 6 # 6-phase tripod gait
		
		thetas_stance_PWM = PWM[0]
		thetas_raise_PWM = PWM[1]
		thetas_swing_PWM = PWM[2]
		thetas_plant_PWM = PWM[3]
		
	
	# Compute length of PWM arrays (assuming computation for any loaded
	# theta arrays returns identical result for now).
	L = len(thetas_stance_PWM['theta1_R1'])
	
	# Obtain step size and 'step' value from file (assume same values for
	# each gait phase for now).
	dt = thetas_stance_PWM['dt']
	step = thetas_stance_PWM['step']
	
	# ---Initialize leg angles to start of gait if init_only=True
	# *** Work in progress ***
	# This needs to be redone after error found, for now use reset_config saved when creating gait with demo file
	if init_only:
		# Compute theta relations for raising/planting legs with vertical foot path
		[theta1_raise, theta2_raise, theta3_raise] = linear_foot_path_solver(robot.robot_dims, [10, -55, -55], 0, 0, 20, 10001, 0.0001)
		thetas_raise = [theta2_raise, theta3_raise]
		
		# Convert to PWM
		thetas_raise_init_PWM = dict()
		for identity in robot.identities:
			for k in range(2):
				thetas_raise_init_PWM['theta' + str(k + 2) + '_' + identity] = np.zeros([len(theta1_raise[0::1000]), ], dtype=int)
				for m, n in enumerate(thetas_raise[k][0:10001:1000]):
					if k != 1:
						thetas_raise_init_PWM['theta' + str(k + 2) + '_' + identity][m] = int(int(math.ceil((robot.center_pos_info[identity][k + 1] + 10*n) / 1))*1)
						
					else:
						thetas_raise_init_PWM['theta' + str(k + 2) + '_' + identity][m] = int(int(math.ceil((robot.center_pos_info[identity][k + 1] - 10*(n + robot.robot_dims['offset_angle'])) / 1))*1)
						
					
					
						
		for leg in robot.legs:
			# Raise leg
			for k in range(1, L):
				robot.move_to([leg.knee_servo, leg.ankle_servo], [thetas_raise_init_PWM['theta2_' + leg.identity][k].item(), thetas_raise_init_PWM['theta3_' + leg.identity][k].item()], [0]*2, [0]*2)
				time.sleep(dt*step*multiplier)
				
			time.sleep(0.25)
			# Move hip servo
			if leg.identity in ['R1', 'L2', 'R3']:
				leg.base_servo.move_to(thetas_stance_PWM['theta1_' + leg.identity][0], 40, 0)
			elif leg.identity in ['L1', 'R2', 'L3']:
				leg.base_servo.move_to(thetas_swing_PWM['theta1_' + leg.identity][0], 40, 0)
				
			time.sleep(0.25)
			
			# Plant leg
			for k in range(1, L):
				robot.move_to([leg.knee_servo, leg.ankle_servo], [thetas_raise_init_PWM['theta2_' + leg.identity][L - 1 - k].item(), thetas_raise_init_PWM['theta3_' + leg.identity][L - 1 - k].item()], [0]*2, [0]*2)
				time.sleep(dt*step*multiplier)
		
			time.sleep(0.25)	
				
				
	
	# ---Peform tripod gait for specified number of steps if init_only=False
	if not init_only:
		if n_phases == 4:
			# Define lists of servos for each phase of tripod gait
			servos_phase_I = robot.servos
			servos_phase_II = list(itertools.chain(*([leg.knee_servo, leg.ankle_servo] for leg in [robot.leg_L1, robot.leg_R2, robot.leg_L3])))
			servos_phase_III = robot.servos
			servos_phase_IV = list(itertools.chain(*([leg.knee_servo, leg.ankle_servo] for leg in [robot.leg_R1, robot.leg_L2, robot.leg_R3])))
			
			# Define lists of PWM signals for each phase of tripod gait
			phase_I_dict = {"L1": thetas_swing_PWM, "R1": thetas_stance_PWM, "L2": thetas_stance_PWM, "R2": thetas_swing_PWM, "L3": thetas_swing_PWM, "R3": thetas_stance_PWM}
			phase_II_dict = {"L1": thetas_plant_PWM, "R2": thetas_plant_PWM, "L3": thetas_plant_PWM}
			phase_III_dict = {"L1": thetas_stance_PWM, "R1": thetas_swing_PWM, "L2": thetas_swing_PWM, "R2": thetas_stance_PWM, "L3": thetas_stance_PWM, "R3": thetas_swing_PWM}
			phase_IV_dict = {"R1": thetas_plant_PWM, "L2": thetas_plant_PWM, "R3": thetas_plant_PWM}
			
			PWM_phase_I = list(itertools.chain(*[[phase_I_dict[key]['theta' + str(k + 1) + '_' + key] for k in range(3)] for key in list(phase_I_dict.keys())]))
			PWM_phase_II = list(itertools.chain(*[[phase_II_dict[key]['theta' + str(k + 2) + '_' + key] for k in range(2)] for key in list(phase_II_dict.keys())]))
			PWM_phase_III = list(itertools.chain(*[[phase_III_dict[key]['theta' + str(k + 1) + '_' + key] for k in range(3)] for key in list(phase_III_dict.keys())]))
			PWM_phase_IV = list(itertools.chain(*[[phase_IV_dict[key]['theta' + str(k + 2) + '_' + key] for k in range(2)] for key in list(phase_IV_dict.keys())]))
			
			# Set all channel speeds to 0
			robot.set_speeds(robot.servos, [0]*18)
			
			delay_gait = 0.2
			# Start gait
			for _ in range(n_steps):
				# Phase I
				for k in range(1, L):
					robot.move_to(servos_phase_I, [PWM_phase_I[m][k].item() for m in range(len(servos_phase_I))], [0]*len(servos_phase_I), [0]*len(servos_phase_I), target_only=True, use_multi_target=True)
					time.sleep(dt*step*multiplier)
					
				time.sleep(delay_gait)
				
				# Phase II
				for k in range(1, L):
					robot.move_to(servos_phase_II, [PWM_phase_II[m][k].item() for m in range(len(servos_phase_II))], [0]*len(servos_phase_II), [0]*len(servos_phase_II), target_only=True, use_multi_target=False)
					time.sleep(dt*step*multiplier)
					
				time.sleep(delay_gait)
				
				# Phase III
				for k in range(1, L):
					robot.move_to(servos_phase_III, [PWM_phase_III[m][k].item() for m in range(len(servos_phase_III))], [0]*len(servos_phase_III), [0]*len(servos_phase_III), target_only=True, use_multi_target=True)
					time.sleep(dt*step*multiplier)
					
				time.sleep(delay_gait)
				
				# Phase IV
				for k in range(1, L):
					robot.move_to(servos_phase_IV, [PWM_phase_IV[m][k].item() for m in range(len(servos_phase_IV))], [0]*len(servos_phase_IV), [0]*len(servos_phase_IV), target_only=True, use_multi_target=False)
					time.sleep(dt*step*multiplier)
					
				time.sleep(delay_gait)
				
		elif n_phases == 6:
			# Define lists of servos for each phase of tripod gait
			servos_phase_I = list(itertools.chain(*([leg.knee_servo, leg.ankle_servo] for leg in [robot.leg_L1, robot.leg_R2, robot.leg_L3])))
			servos_phase_II = robot.servos
			servos_phase_III = list(itertools.chain(*([leg.knee_servo, leg.ankle_servo] for leg in [robot.leg_L1, robot.leg_R2, robot.leg_L3])))
			servos_phase_IV = list(itertools.chain(*([leg.knee_servo, leg.ankle_servo] for leg in [robot.leg_R1, robot.leg_L2, robot.leg_R3])))
			servos_phase_V = robot.servos
			servos_phase_VI = list(itertools.chain(*([leg.knee_servo, leg.ankle_servo] for leg in [robot.leg_R1, robot.leg_L2, robot.leg_R3])))
			
			# Define lists of PWM signals for each phase of tripod gait
			phase_I_dict = {"L1": thetas_raise_PWM, "R2": thetas_raise_PWM, "L3": thetas_raise_PWM}
			phase_II_dict = {"L1": thetas_swing_PWM, "R1": thetas_stance_PWM, "L2": thetas_stance_PWM, "R2": thetas_swing_PWM, "L3": thetas_swing_PWM, "R3": thetas_stance_PWM}
			phase_III_dict = {"L1": thetas_plant_PWM, "R2": thetas_plant_PWM, "L3": thetas_plant_PWM}
			phase_IV_dict = {"R1": thetas_raise_PWM, "L2": thetas_raise_PWM, "R3": thetas_raise_PWM}
			phase_V_dict = {"L1": thetas_stance_PWM, "R1": thetas_swing_PWM, "L2": thetas_swing_PWM, "R2": thetas_stance_PWM, "L3": thetas_stance_PWM, "R3": thetas_swing_PWM}
			phase_VI_dict = {"R1": thetas_plant_PWM, "L2": thetas_plant_PWM, "R3": thetas_plant_PWM}
			
			PWM_phase_I = list(itertools.chain(*[[phase_I_dict[key]['theta' + str(k + 2) + '_' + key] for k in range(2)] for key in list(phase_I_dict.keys())]))
			PWM_phase_II = list(itertools.chain(*[[phase_II_dict[key]['theta' + str(k + 1) + '_' + key] for k in range(3)] for key in list(phase_II_dict.keys())]))
			PWM_phase_III = list(itertools.chain(*[[phase_III_dict[key]['theta' + str(k + 2) + '_' + key] for k in range(2)] for key in list(phase_III_dict.keys())]))
			PWM_phase_IV = list(itertools.chain(*[[phase_IV_dict[key]['theta' + str(k + 2) + '_' + key] for k in range(2)] for key in list(phase_IV_dict.keys())]))
			PWM_phase_V = list(itertools.chain(*[[phase_V_dict[key]['theta' + str(k + 1) + '_' + key] for k in range(3)] for key in list(phase_V_dict.keys())]))
			PWM_phase_VI = list(itertools.chain(*[[phase_VI_dict[key]['theta' + str(k + 2) + '_' + key] for k in range(2)] for key in list(phase_VI_dict.keys())]))
			
			# Set all channel speeds to 0
			robot.set_speeds(robot.servos, [0]*18)
			
			delay_gait = 0.2
			# Start gait
			for _ in range(n_steps):
				# Phase I
				for k in range(1, L):
					robot.move_to(servos_phase_I, [PWM_phase_I[m][k].item() for m in range(len(servos_phase_I))], [0]*len(servos_phase_I), [0]*len(servos_phase_I), target_only=True, use_multi_target=False)
					time.sleep(dt*step*multiplier)
					
				time.sleep(delay_gait)
				
				# Phase II
				for k in range(1, L):
					robot.move_to(servos_phase_II, [PWM_phase_II[m][k].item() for m in range(len(servos_phase_II))], [0]*len(servos_phase_II), [0]*len(servos_phase_II), target_only=True, use_multi_target=True)
					time.sleep(dt*step*multiplier)
					
				time.sleep(delay_gait)
				
				# Phase III
				for k in range(1, L):
					robot.move_to(servos_phase_III, [PWM_phase_III[m][k].item() for m in range(len(servos_phase_III))], [0]*len(servos_phase_III), [0]*len(servos_phase_III), target_only=True, use_multi_target=False)
					time.sleep(dt*step*multiplier)
					
				time.sleep(delay_gait)
				
				# Phase IV
				for k in range(1, L):
					robot.move_to(servos_phase_IV, [PWM_phase_IV[m][k].item() for m in range(len(servos_phase_IV))], [0]*len(servos_phase_IV), [0]*len(servos_phase_IV), target_only=True, use_multi_target=False)
					time.sleep(dt*step*multiplier)
					
				time.sleep(delay_gait)
				
				# Phase V
				for k in range(1, L):
					robot.move_to(servos_phase_V, [PWM_phase_V[m][k].item() for m in range(len(servos_phase_V))], [0]*len(servos_phase_V), [0]*len(servos_phase_V), target_only=True, use_multi_target=True)
					time.sleep(dt*step*multiplier)
					
				time.sleep(delay_gait)
				
				# Phase VI
				for k in range(1, L):
					robot.move_to(servos_phase_VI, [PWM_phase_VI[m][k].item() for m in range(len(servos_phase_VI))], [0]*len(servos_phase_VI), [0]*len(servos_phase_VI), target_only=True, use_multi_target=False)
					time.sleep(dt*step*multiplier)
					
				time.sleep(delay_gait)
				



def create_pull_inward(gait_params, robot_dims):
	###
	#
	# gait_params:
	# robot_dims:
	#
	# returns: 
	
	# Unpack parameter values from gait_params
	vx = gait_params['vx']
	vy = gait_params['vy']
	vh = gait_params['vh']
	points = gait_params['points']
	dt = gait_params['dt']
	
	# ---Define list of leg identities for simplifying following code
	identities = ['R1', 'L2', 'R3', 'L1', 'R2', 'L3']
	
	# ---Compute theta relations during stance movement
	thetas_stance = dict()

	# Define foot path paratmeters for each leg of tripod (assume right tripod)
	# Format: [robot_dims, [theta1, theta2, theta3], vx, vy, vh, points, dt]
	params_stance_rf = [robot_dims, gait_params['init_conds_R1'], -1*vx, -1*vy, 0, points, dt]	# Front tripod leg (R1)
	params_stance_lm = [robot_dims, gait_params['init_conds_L2'], -1*vx*np.sqrt(2), 0, 0, points, dt]	# Middle tripod leg (L2)
	params_stance_rb = [robot_dims, gait_params['init_conds_R3'], -1*vx, vy, 0, points, dt] # Back tripod leg (R3)
	params_stance = [params_stance_rf, params_stance_lm, params_stance_rb]

	# Compute thetas for right tripod using linear_foot_path_solver() and assign
	# values for left tripod based on symmetric tripod gait
	for k in range(3):
		thetas_stance['theta1_' + identities[k]], thetas_stance['theta2_' + identities[k]], \
			thetas_stance['theta3_' + identities[k]] = linear_foot_path_solver(*params_stance[k])
			
		thetas_stance['theta1_' + identities[k + 3]] = -1*thetas_stance['theta1_' + identities[k]]
		thetas_stance['theta2_' + identities[k + 3]] = thetas_stance['theta2_' + identities[k]]
		thetas_stance['theta3_' + identities[k + 3]] = thetas_stance['theta3_' + identities[k]]
		
	thetas_stance['points'] = points
	thetas_stance['dt'] = dt

	# Plot resulitng theta vs. time relations for stance phase
	thetas_plotter(thetas_stance)
	
	# ---Compute theta relations during swing movement (4-phase) or raising
	# movement (6-phase)
	# Define foot path paratmeters for each leg of tripod
	thetas_swing = dict()
	params_swing_rf = [robot_dims, [thetas_stance['theta1_R1'][-1], thetas_stance['theta2_R1'][-1], \
					thetas_stance['theta3_R1'][-1]], vx, vy, vh, points, dt]	# Front tripod leg (rf)
	params_swing_lm = [robot_dims, [thetas_stance['theta1_L2'][-1], thetas_stance['theta2_L2'][-1], \
					thetas_stance['theta3_L2'][-1]], vx*np.sqrt(2), 0, vh, points, dt]	# Middle tripod leg (lm)
	params_swing_rb = [robot_dims, [thetas_stance['theta1_R3'][-1], thetas_stance['theta2_R3'][-1], \
					thetas_stance['theta3_R3'][-1]], vx, -1*vy, vh, points, dt] # Back tripod leg (rb)
					
	params_swing = [params_swing_rf, params_swing_lm, params_swing_rb]

	# Compute thetas for right tripod using linear_foot_path_solver() and assign
	# values for left tripod based on symmetric tripod gait
	for k in range(3): # 6-phase
		thetas_swing['theta1_' + identities[k]], thetas_swing['theta2_' + identities[k]], \
			thetas_swing['theta3_' + identities[k]] = linear_foot_path_solver(*params_swing[k])
			
		thetas_swing['theta1_' + identities[k + 3]] = -1*thetas_swing['theta1_' + identities[k]]
		thetas_swing['theta2_' + identities[k + 3]] = thetas_swing['theta2_' + identities[k]]
		thetas_swing['theta3_' + identities[k + 3]] = thetas_swing['theta3_' + identities[k]]
		
	thetas_swing['points'] = points
	thetas_swing['dt'] = dt
	
	# Plot resulitng theta vs. time relations
	thetas_plotter(thetas_swing)
	
	# ---Compute theta relations during plant movement (4-phase) or swing
	# movement (6-phase)
	# Define foot path paratmeters for each leg of tripod
	thetas_plant = dict()
	params_plant_rf = [robot_dims, [thetas_swing['theta1_R1'][-1], thetas_swing['theta2_R1'][-1], \
					thetas_swing['theta3_R1'][-1]], 0, 0, -1*vh, points, dt]	# Front tripod leg (rf)
	params_plant_lm = [robot_dims, [thetas_swing['theta1_L2'][-1], thetas_swing['theta2_L2'][-1], \
					thetas_swing['theta3_L2'][-1]], 0, 0, -1*vh, points, dt]	# Middle tripod leg (lm)
	params_plant_rb = [robot_dims, [thetas_swing['theta1_R3'][-1], thetas_swing['theta2_R3'][-1], \
					thetas_swing['theta3_R3'][-1]], 0, 0, -1*vh, points, dt] # Back tripod leg (rb)
	params_plant = [params_plant_rf, params_plant_lm, params_plant_rb]	

	# Compute thetas for right tripod using linear_foot_path_solver() and assign
	# values for left tripod based on symmetric tripod gait
	for k in range(3):
		thetas_plant['theta1_' + identities[k]], thetas_plant['theta2_' + identities[k]], \
			thetas_plant['theta3_' + identities[k]] = linear_foot_path_solver(*params_plant[k])
			
		thetas_plant['theta1_' + identities[k + 3]] = -1*thetas_plant['theta1_' + identities[k]]
		thetas_plant['theta2_' + identities[k + 3]] = thetas_plant['theta2_' + identities[k]]
		thetas_plant['theta3_' + identities[k + 3]] = thetas_plant['theta3_' + identities[k]]
		
	thetas_plant['points'] = points
	thetas_plant['dt'] = dt

	# Plot resulitng theta vs. time relations
	thetas_plotter(thetas_plant)
		
	thetas = [thetas_stance, thetas_swing, thetas_plant]
		
	return thetas

def pull_inward(robot, PWM, n_steps, multiplier=1/6):
	# This function implements the pull inward given computed values of
	# theta vs. time relations for this motion.
	#
	# robot:		Instance of Sebastian class
	# PWM:			Python list of PWM relations for tripod gait [stance, swing, plant]
	# n_steps:		Number of steps of tripod gait
	# multiplier:	Parameter for determining time between successive target serial commands, default is 1/6
	
	# --- Initialize PWM vs. time relations
	thetas_stance_PWM = PWM[0]
	thetas_swing_PWM = PWM[1]
	thetas_plant_PWM = PWM[2]
	
	# Compute length of PWM arrays (assuming computation for any loaded
	# theta arrays returns identical result for now).
	L = len(thetas_stance_PWM['theta1_R1'])
	
	# Obtain step size and 'step' value from file (assume same values for
	# each gait phase for now).
	dt = thetas_stance_PWM['dt']
	step = thetas_stance_PWM['step']
	
	# ---Peform tripod gait for specified number of steps if init_only=False
	# Define lists of servos for each phase of tripod gait
	servos_phase_I = list(itertools.chain(*([leg.knee_servo, leg.ankle_servo] for leg in [robot.leg_R1, robot.leg_L2, robot.leg_R3])))
	servos_phase_II = list(itertools.chain(*([leg.knee_servo, leg.ankle_servo] for leg in [robot.leg_R1, robot.leg_L2, robot.leg_R3])))
	servos_phase_III = list(itertools.chain(*([leg.knee_servo, leg.ankle_servo] for leg in [robot.leg_R1, robot.leg_L2, robot.leg_R3])))
	servos_phase_IV = list(itertools.chain(*([leg.knee_servo, leg.ankle_servo] for leg in [robot.leg_L1, robot.leg_R2, robot.leg_L3])))
	servos_phase_V = list(itertools.chain(*([leg.knee_servo, leg.ankle_servo] for leg in [robot.leg_L1, robot.leg_R2, robot.leg_L3])))
	servos_phase_VI = list(itertools.chain(*([leg.knee_servo, leg.ankle_servo] for leg in [robot.leg_L1, robot.leg_R2, robot.leg_L3])))
	
	# Define lists of PWM signals for each phase of tripod gait
	phase_I_dict = {"R1": thetas_stance_PWM, "L2": thetas_stance_PWM, "R3": thetas_stance_PWM}
	phase_II_dict = {"R1": thetas_swing_PWM, "L2": thetas_swing_PWM, "R3": thetas_swing_PWM}
	phase_III_dict = {"R1": thetas_plant_PWM, "L2": thetas_plant_PWM, "R3": thetas_plant_PWM}
	phase_IV_dict = {"L1": thetas_stance_PWM, "R2": thetas_stance_PWM, "L3": thetas_stance_PWM}
	phase_V_dict = {"L1": thetas_swing_PWM, "R2": thetas_swing_PWM, "L3": thetas_swing_PWM}
	phase_VI_dict = {"L1": thetas_plant_PWM, "R2": thetas_plant_PWM, "L3": thetas_plant_PWM}
	
	PWM_phase_I = list(itertools.chain(*[[phase_I_dict[key]['theta' + str(k + 2) + '_' + key] for k in range(2)] for key in list(phase_I_dict.keys())]))
	PWM_phase_II = list(itertools.chain(*[[phase_II_dict[key]['theta' + str(k + 2) + '_' + key] for k in range(2)] for key in list(phase_II_dict.keys())]))
	PWM_phase_III = list(itertools.chain(*[[phase_III_dict[key]['theta' + str(k + 2) + '_' + key] for k in range(2)] for key in list(phase_III_dict.keys())]))
	PWM_phase_IV = list(itertools.chain(*[[phase_IV_dict[key]['theta' + str(k + 2) + '_' + key] for k in range(2)] for key in list(phase_IV_dict.keys())]))
	PWM_phase_V = list(itertools.chain(*[[phase_V_dict[key]['theta' + str(k + 2) + '_' + key] for k in range(2)] for key in list(phase_V_dict.keys())]))
	PWM_phase_VI = list(itertools.chain(*[[phase_VI_dict[key]['theta' + str(k + 2) + '_' + key] for k in range(2)] for key in list(phase_VI_dict.keys())]))
	
	# Set all channel speeds to 0
	robot.set_speeds(robot.servos, [0]*18)
	
	delay_gait = 0.2
	# Start gait
	for _ in range(n_steps):
		# Phase I
		for k in range(1, L):
			robot.move_to(servos_phase_I, [PWM_phase_I[m][k].item() for m in range(len(servos_phase_I))], [0]*len(servos_phase_I), [0]*len(servos_phase_I), target_only=True, use_multi_target=False)
			time.sleep(dt*step*multiplier)
			
		time.sleep(delay_gait)
		
		# Phase II
		for k in range(1, L):
			robot.move_to(servos_phase_II, [PWM_phase_II[m][k].item() for m in range(len(servos_phase_II))], [0]*len(servos_phase_II), [0]*len(servos_phase_II), target_only=True, use_multi_target=False)
			time.sleep(dt*step*multiplier)
			
		time.sleep(delay_gait)
		
		# Phase III
		for k in range(1, L):
			robot.move_to(servos_phase_III, [PWM_phase_III[m][k].item() for m in range(len(servos_phase_III))], [0]*len(servos_phase_III), [0]*len(servos_phase_III), target_only=True, use_multi_target=False)
			time.sleep(dt*step*multiplier)
			
		time.sleep(delay_gait)
		
		# Phase IV
		for k in range(1, L):
			robot.move_to(servos_phase_IV, [PWM_phase_IV[m][k].item() for m in range(len(servos_phase_IV))], [0]*len(servos_phase_IV), [0]*len(servos_phase_IV), target_only=True, use_multi_target=False)
			time.sleep(dt*step*multiplier)
			
		time.sleep(delay_gait)
		
		# Phase V
		for k in range(1, L):
			robot.move_to(servos_phase_V, [PWM_phase_V[m][k].item() for m in range(len(servos_phase_V))], [0]*len(servos_phase_V), [0]*len(servos_phase_V), target_only=True, use_multi_target=False)
			time.sleep(dt*step*multiplier)
			
		time.sleep(delay_gait)
		
		# Phase VI
		for k in range(1, L):
			robot.move_to(servos_phase_VI, [PWM_phase_VI[m][k].item() for m in range(len(servos_phase_VI))], [0]*len(servos_phase_VI), [0]*len(servos_phase_VI), target_only=True, use_multi_target=False)
			time.sleep(dt*step*multiplier)
			
		time.sleep(delay_gait)
		
		time.sleep(5)
		
	
	return
