### Author: Nicole Graf (3/14/2021)
###
### ---This file defines the center positions (in pulse width, us) and
### servo pin numbers when connecting to the Pololu Maestro servo controller,
### as well as several reset configurations. Update values in this file whenever
### they are changed.
###
### ---Last updated: 3/14/2021

import numpy as np

import math

# --- Define physical robot dimensions
# All lengths in millimeters
robot_dims = dict()

base_mm = 172.92						# Distance between hip axes of left and right sides
base_attach_to_base_joint_mm = 41.24	# Distance from base edge to hip axis
base_joint_to_knee_joint_mm = 89.38		# Length of leg segment "L0"
knee_joint_to_ankle_joint_mm = 75.06	# Length of leg segment "L1"
ankle_joint_to_dactyl_mm = 70.59		# Distance from ankle axis to beginning of dactyl
dactyl_dx_mm = 49.29					# Horizontal distance from center of dactyl start to center of dactyl tip
dactyl_dy_mm = 14.26					# Vertical distance from center of dactyl start to center of dactyl tip
dactyl_center_offset_mm = 1				# Vertical distance between ankle servo shaft center and dactyl base center
dactyl_angle = math.atan2(dactyl_dy_mm, dactyl_dx_mm)
robot_dims['L0'] = base_joint_to_knee_joint_mm
robot_dims['L1'] = knee_joint_to_ankle_joint_mm
# Compute L2 dimension from law of cosines
a = dactyl_center_offset_mm/(np.sin(math.atan2(dactyl_center_offset_mm/ankle_joint_to_dactyl_mm, 1)))
b = np.sqrt(pow(dactyl_dx_mm, 2) + pow(dactyl_dy_mm, 2))
gamma = np.pi - math.atan2(dactyl_center_offset_mm/ankle_joint_to_dactyl_mm, 1) - dactyl_angle
robot_dims['L2'] = np.sqrt(pow(a, 2) + pow(b, 2) - 2*a*b*np.cos(gamma))
# Compute angle offset from L2 dimension to actual ankle servo angle
robot_dims['offset_angle'] = (np.arccos((pow(a, 2) - pow(b, 2) - pow(robot_dims['L2'], 2))/(-2*robot_dims['L2']*b)) - math.atan2(dactyl_center_offset_mm/ankle_joint_to_dactyl_mm, 1))*(180/np.pi)

# --- Define servo center positions
# Defined as pulse width at which leg segments attached to servo are parallel
# Format: [base, knee, ankle]
center_pos_info= dict()
center_pos_info['L1'] = [1940, 1925, 1015]
center_pos_info['R1'] = [1055, 1910, 1050]
center_pos_info['L2'] = [1540, 1940, 1140]
center_pos_info['R2'] = [1620, 1975, 1090]
center_pos_info['L3'] = [1050, 1890, 1000]
center_pos_info['R3'] = [1980, 1985, 1140]

# --- Define pin number on Maestro for each servo
pins_info = dict()
pins_info['L1'] = [0, 1, 2]
pins_info['R1'] = [3, 4, 5]
pins_info['L2'] = [6, 7, 8]
pins_info['R2'] = [9, 10, 11]
pins_info['L3'] = [12, 13, 14]
pins_info['R3'] = [15, 16, 17]

# --- Define servo reset positions configuration #1
# Description: all base servos straight, knee, ankle servos 45 degrees
reset1_pos_info = dict()
reset1_pos_info['L1'] = [center_pos_info['L1'][0], center_pos_info['L1'][1] - 450, center_pos_info['L1'][2] + 450]
reset1_pos_info['R1'] = [center_pos_info['R1'][0], center_pos_info['R1'][1] - 450, center_pos_info['R1'][2] + 450]
reset1_pos_info['L2'] = [center_pos_info['L2'][0], center_pos_info['L2'][1] - 450, center_pos_info['L2'][2] + 450]
reset1_pos_info['R2'] = [center_pos_info['R2'][0], center_pos_info['R2'][1] - 450, center_pos_info['R2'][2] + 450]
reset1_pos_info['L3'] = [center_pos_info['L3'][0], center_pos_info['L3'][1] - 450, center_pos_info['L3'][2] + 450]
reset1_pos_info['R3'] = [center_pos_info['R3'][0], center_pos_info['R3'][1] - 450, center_pos_info['R3'][2] + 450]

# --- Define servo reset positions configuration #2
# Description: outer base servos 45 degrees outwards, knee, ankle servos 45 degrees
reset2_pos_info = dict()
reset2_pos_info['L1'] = [center_pos_info['L1'][0] - 450, center_pos_info['L1'][1] - 450, center_pos_info['L1'][2] + 450]
reset2_pos_info['R1'] = [center_pos_info['R1'][0] + 450, center_pos_info['R1'][1] - 450, center_pos_info['R1'][2] + 450]
reset2_pos_info['L2'] = [center_pos_info['L2'][0], center_pos_info['L2'][1] - 450, center_pos_info['L2'][2] + 450]
reset2_pos_info['R2'] = [center_pos_info['R2'][0], center_pos_info['R2'][1] - 450, center_pos_info['R2'][2] + 450]
reset2_pos_info['L3'] = [center_pos_info['L3'][0] + 450, center_pos_info['L3'][1] - 450, center_pos_info['L3'][2] + 450]
reset2_pos_info['R3'] = [center_pos_info['R3'][0] - 450, center_pos_info['R3'][1] - 450, center_pos_info['R3'][2] + 450]

# --- Define servo reset positions configuration #3 
# Description: outer base servos 45 degrees outwards, knee servos 55 degrees, ankle servos 55 degrees
reset3_pos_info = dict()
reset3_pos_info['L1'] = [center_pos_info['L1'][0] - 450, center_pos_info['L1'][1] - 550, center_pos_info['L1'][2] + 550]
reset3_pos_info['R1'] = [center_pos_info['R1'][0] + 450, center_pos_info['R1'][1] - 550, center_pos_info['R1'][2] + 550]
reset3_pos_info['L2'] = [center_pos_info['L2'][0], center_pos_info['L2'][1] - 550, center_pos_info['L2'][2] + 550]
reset3_pos_info['R2'] = [center_pos_info['R2'][0], center_pos_info['R2'][1] - 550, center_pos_info['R2'][2] + 550]
reset3_pos_info['L3'] = [center_pos_info['L3'][0] + 450, center_pos_info['L3'][1] - 550, center_pos_info['L3'][2] + 550]
reset3_pos_info['R3'] = [center_pos_info['R3'][0] - 450, center_pos_info['R3'][1] - 550, center_pos_info['R3'][2] + 550]

# --- Define servo reset positions configuration #4
# Description: outer base servos 30 degrees outwards, knee, ankle servos 45 degrees
reset4_pos_info = dict()
reset4_pos_info['L1'] = [center_pos_info['L1'][0] - 300, center_pos_info['L1'][1] - 450, center_pos_info['L1'][2] + 450]
reset4_pos_info['R1'] = [center_pos_info['R1'][0] + 300, center_pos_info['R1'][1] - 450, center_pos_info['R1'][2] + 450]
reset4_pos_info['L2'] = [center_pos_info['L2'][0], center_pos_info['L2'][1] - 450, center_pos_info['L2'][2] + 450]
reset4_pos_info['R2'] = [center_pos_info['R2'][0], center_pos_info['R2'][1] - 450, center_pos_info['R2'][2] + 450]
reset4_pos_info['L3'] = [center_pos_info['L3'][0] + 300, center_pos_info['L3'][1] - 450, center_pos_info['L3'][2] + 450]
reset4_pos_info['R3'] = [center_pos_info['R3'][0] - 300, center_pos_info['R3'][1] - 450, center_pos_info['R3'][2] + 450]

# --- Define servo reset positions configuration #5
# Description: outer base servos 45 degrees outwards, knee servos 55 degrees, ankle servos 55 degrees with offset
reset5_pos_info = dict()
reset5_pos_info['L1'] = [center_pos_info['L1'][0] - 450, center_pos_info['L1'][1] - 550, center_pos_info['L1'][2] + 550 - int(10*robot_dims['offset_angle'])]
reset5_pos_info['R1'] = [center_pos_info['R1'][0] + 450, center_pos_info['R1'][1] - 550, center_pos_info['R1'][2] + 550 - int(10*robot_dims['offset_angle'])]
reset5_pos_info['L2'] = [center_pos_info['L2'][0], center_pos_info['L2'][1] - 550, center_pos_info['L2'][2] + 550 - int(10*robot_dims['offset_angle'])]
reset5_pos_info['R2'] = [center_pos_info['R2'][0], center_pos_info['R2'][1] - 550, center_pos_info['R2'][2] + 550 - int(10*robot_dims['offset_angle'])]
reset5_pos_info['L3'] = [center_pos_info['L3'][0] + 450, center_pos_info['L3'][1] - 550, center_pos_info['L3'][2] + 550 - int(10*robot_dims['offset_angle'])]
reset5_pos_info['R3'] = [center_pos_info['R3'][0] - 450, center_pos_info['R3'][1] - 550, center_pos_info['R3'][2] + 550 - int(10*robot_dims['offset_angle'])]
