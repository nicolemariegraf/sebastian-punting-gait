### Author: John Grezmak (1/23/20)
###
### ---This library defines motion functions for a Sebastian robot using
### the 18-pin Pololu Maestro as the servo controller.
###
### ---Last updated 11/5/20

import itertools
import pickle
import time

import numpy as np

import maestro_serial

class Sebastian():
	def __init__(self, center_pos_info, pins_info, reset_info, robot_dims, last_pos_file=None):
		self.center_pos_info = center_pos_info
		self.pins_info = pins_info
		self.reset_info = reset_info
		self.robot_dims = robot_dims
		self.last_pos_file = last_pos_file
		
		# Create instance of Maestro class
		self.mu = maestro_serial.Maestro(device='/dev/ttyACM0', baudrate=115200)
		
		# Define list of leg identities
		self.identities = ["L1", "R1", "L2", "R2", "L3", "R3"]
		
		# Create instance of leg class for each leg
		self.leg_L1 = leg("L1", self.center_pos_info['L1'], self.pins_info['L1'], self.reset_info['L1'], self.mu)
		self.leg_R1 = leg("R1", self.center_pos_info['R1'], self.pins_info['R1'], self.reset_info['R1'], self.mu)
		self.leg_L2 = leg("L2", self.center_pos_info['L2'], self.pins_info['L2'], self.reset_info['L2'], self.mu)
		self.leg_R2 = leg("R2", self.center_pos_info['R2'], self.pins_info['R2'], self.reset_info['R2'], self.mu)
		self.leg_L3 = leg("L3", self.center_pos_info['L3'], self.pins_info['L3'], self.reset_info['L3'], self.mu)
		self.leg_R3 = leg("R3", self.center_pos_info['R3'], self.pins_info['R3'], self.reset_info['R3'], self.mu)
		
		# Define lists of legs and servos/servo types
		self.legs = [self.leg_L1, self.leg_R1, self.leg_L2, self.leg_R2, self.leg_L3, self.leg_R3]
		self.servos = list(itertools.chain(*(leg.servos for leg in self.legs)))
		self.base_servos = list(itertools.chain(*([leg.base_servo, ] for leg in self.legs)))
		self.knee_servos = list(itertools.chain(*([leg.knee_servo, ] for leg in self.legs)))
		self.ankle_servos = list(itertools.chain(*([leg.ankle_servo, ] for leg in self.legs)))
		
	def reset(self, mode=1):
		### Sebastian class-level reset function. Note that all servos have
		### speed and acceleration set to 32 and 0, respectively, regardless
		### of reset mode
		# mode:		reset mode, default is 1 (blind reset)
		
		if mode == 1:	# Use if all servos have NOT moved since last powered
			print('Resetting with mode 1 (blind reset)...')
			for servo in self.servos:
				servo.reset(32, 0)
				time.sleep(0.9)
				
			print('\n')
			time.sleep(1)
		elif mode == 2: # Use if all servos HAVE moved since last powered
			print('Resetting with mode 2 (synchronous reset)...')
			# Reset base servos
			for servo in self.base_servos:
				servo.reset(32, 0)
				
			time.sleep(1.5)
			
			# Reset knee servos
			for servo in self.knee_servos:
				servo.reset(32, 0)
				
			time.sleep(1.5)
			
			# Reset ankle servos
			for servo in self.ankle_servos:
				servo.reset(32, 0)
				
			print('\n')
			time.sleep(1.5)
			
			
			
	def move_by(self, servos, target_increments, speeds, accels):
		### Sebastian class-level move_by function
		# servos:				list of servos to move
		# target_increments:	list of target increments for servos in "servos"
		# speeds:				list of speeds for servos in "servos"
		# accels:				list of accelerations for servos in "servos"
		
		for k, servo in enumerate(servos):
			servo.mu.set_speed(servo.pin, speeds[k])
			servo.mu.set_acceleration(servo.pin, accels[k])
			
		for k, servo in enumerate(servos):
			servo.mu.set_target(servo.pin, (servo.pos + target_increments[k])*4)
			
		for k, servo in enumerate(servos):
			servo.pos = servo.pos + target_increments[k]
			
			
			
	def move_to(self, servos, targets, speeds, accels, target_only=False, use_multi_target=False):
		### Sebastian class-level move_by function
		# servos:				list of servos to move
		# target_increments:	list of targets for servos in "servos"
		# speeds:				list of speeds for servos in "servos"
		# accels:				list of accelerations for servos in "servos"
		# target_only:			if True (bool), only target command is sent
		# use_multi_target:		if True (bool), uses "set multiple targets" serial command instead of setting targets individually
		
		if not target_only:
			for k, servo in enumerate(servos):
				servo.mu.set_speed(servo.pin, speeds[k])
				servo.mu.set_acceleration(servo.pin, accels[k])
				
			
		# Multiply target values by 4
		targets = [target*4 for target in targets]
		
		if not use_multi_target:
			# Each target set using individual serial command
			for k, servo in enumerate(servos):
				servo.mu.set_target(servo.pin, targets[k])
				
			for k, servo in enumerate(servos):
				servo.pos = targets[k]
					
		else:
			# Multiple targets set using one serial command
			
			# Determine starting pin number
			channel = min(servo.pin for servo in servos)
			
			# Set targets use set_multiple_targets serial command
			self.mu.set_multiple_targets(channel, targets)
			
			
	def set_speeds(self, servos, speeds):
		### Sebastian class-level set_speed function
		
		for k, servo in enumerate(servos):
			servo.mu.set_speed(servo.pin, speeds[k])
			
			
	def set_accels(self, servos, accels):
		### Sebastian class-level set_accels function
		
		for k, servo in enumerate(servos):
			servo.mu.set_acceleration(servo.pin, accels[k])
			
	
	def threading_main(self, main_queue, main_gait_func, n_steps):
		### Function for choosing gaits/movements during exploratory search using sensor data
		### Work in progress...
		# main_queue:		Python Queue for sharing variables between threads
		# main_gait_func:	Function for performing one step of the main gait
		
		movement_end = False
		k = 0
		time.sleep(1)
		
		while movement_end == False:
			# Get current version of main dict
			dict_temp = main_queue.get()
			main_queue.put(dict_temp)
			#time.sleep(0.1)
			
			# Specify next move based on current dict contents
			if dict_temp['next_move'] == 'none':
				print('Next move set to "none", ending threading_main thread...')
				movement_end = True
			elif dict_temp['next_move'] == 'main':
				print('Moving additional step using main gait...\n')
				main_gait_func(*dict_temp['gait_params'])	
				
				k = k + 1
				#print(str(k) + ' moves have been made...\n')
				
			
			# Decide what move to make next based on data analysis
			if k == 1:
				print('Movement limit reached...\n')
				movement_end = True
				dict_temp = main_queue.get()
				dict_temp['stop'] = True
				dict_temp['next_move'] = 'none'
				main_queue.put(dict_temp)
				k = k + 1
			else:
				pass
				#time.sleep(1.5)
			
		
	
		return
		
		
		
class leg():
	# Class for each leg of the robot
	def __init__(self, identity, center_pos_info, pins_info, reset_info, mu, last_servo_pos=None):
		self.identity = identity
		self.mu = mu
		
		self.base_servo = servo(self.identity + " base servo", center_pos_info[0], pins_info[0], reset_info[0], self.mu)
		self.knee_servo = servo(self.identity + " knee servo", center_pos_info[1], pins_info[1], reset_info[1], self.mu)
		self.ankle_servo = servo(self.identity + " ankle servo", center_pos_info[2], pins_info[2], reset_info[2], self.mu)
		self.servos = [self.base_servo, self.knee_servo, self.ankle_servo]
		
	def reset(self, speed, accel):
		# Reset all servos in leg
		self.base_servo.reset(speed, accel)
		time.sleep(1)
		self.knee_servo.reset(speed, accel)
		time.sleep(1)
		self.ankle_servo.reset(speed, accel)
		time.sleep(1)
		
		
		
class servo():
	# Class for each servo of the robot
	def __init__(self, identity, center_pos_info, pin_info, reset_info, mu, last_servo_pos=None):
		self.identity = identity
		self.center_pos = center_pos_info
		self.pin = pin_info
		self.reset_pos = reset_info
		self.mu = mu
		self.pos = self.reset_pos
		
	def move_by(self, target_increment, speed, accel):
		# Move servo by specified increment (as pulse width, in quarter-microseconds)
		print('Moving ' + self.identity + ' by ' + str(target_increment) + ' quarter-microseconds.')
		self.mu.set_speed(self.pin, speed)
		self.mu.set_acceleration(self.pin, accel)
		self.mu.set_target(self.pin, (self.pos + target_increment)*4)
		self.pos = self.pos + target_increment
		
	def move_to(self, target, speed, accel, target_only=False):
		# Move servo to position specified by target (as pulse width, in quarter-microseconds)
		#print('Moving ' + self.identity + ' to ' + str(target) + ' quarter-microseconds.')
		if not target_only:
			self.mu.set_speed(self.pin, speed)
			self.mu.set_acceleration(self.pin, accel)
			self.mu.set_target(self.pin, target*4)
			self.pos = target
		else:
			self.mu.set_target(self.pin, target*4)
			self.pos = target
			
		
	def reset(self, speed, accel):
		# Move servo to reset position
		print('Resetting ' + self.identity)
		self.mu.set_speed(self.pin, speed)
		self.mu.set_acceleration(self.pin, accel)
		self.move_to(self.reset_pos, speed, accel)
		self.pos = self.reset_pos
