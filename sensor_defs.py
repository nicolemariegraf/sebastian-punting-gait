### Author: John Grezmak
### Library of functions for sensors used with Sebastian robots.

from multiprocessing.pool import ThreadPool
import pickle
import time

import numpy as np
import serial

def collectData(sensor_type, data_length, ser_ID):
	### Function for collecting data from serial device for specified length of data
	# sensor_type:		Type of sensor; currently supported: "MLX90393", "BNO055"
	# data_length:		Length of data to collect
	# ser_ID:			String specifying serial port for serial device
	
	data = np.zeros([data_length, ])
	if sensor_type == 'MLX90393':
		for k in range(data_length):
			ser_bytes = ser_ID.readline()
			decoded_bytes = float(ser_bytes[0:len(ser_bytes)-2].decode("utf-8"))
			data[k] = decoded_bytes
			print(decoded_bytes)
			
	elif sensor_type == 'BNO055':
		for k in range(data_length + 1):
			ser_bytes = ser_ID.readline()
			if k > 0:
				decoded_bytes = float(ser_bytes[0:len(ser_bytes)-2].decode("utf-8"))
				data[k - 1] = decoded_bytes
				print(decoded_bytes)
				
			
		
		
	return data
	
def initialize_sensor(sensor_type):
	### Function for initializing sensor.
	# sensor_type:		Type of sensor; currently supported: "MLX90393", "BNO055"
	if sensor_type == 'MLX90393':
		initialized = False
		while initialized == False:
			# Set up serial communication with Arduino(s)
			port1 = '/dev/ttyUSB0'
			ser1 = serial.Serial(port1, 115200, timeout=2)
			ser1.flushInput()

			port2 = '/dev/ttyUSB1'
			ser2 = serial.Serial(port2, 115200, timeout=2)
			ser2.flushInput()
			
			pool = ThreadPool(processes=2)
			collectData_result1 = pool.apply_async(collectData, ('MLX90393', 9, ser1))
			collectData_result2 = pool.apply_async(collectData, ('MLX90393', 9, ser2))
			pool.close()
			pool.join()
			#print(collectData_result1.get())
			
			data_error = False
			try:
				temp = collectData_result1.get()
				if 0.0 in temp:
					data_error = True
					print('Error in left tripod sensor(s)')
					
			except:
				data_error = True
				print('Error in left tripod sensor(s)')
				
				
			try:
				temp = collectData_result2.get()
				if 0.0 in temp:
					data_error = True
					print('Error in right tripod sensor(s)')
			except:
				data_error = True
				print('Error in right tripod sensor(s)')
			
			
			if data_error:	
				print('Failed to initialize one or more magnetometers, trying again...')
				ser1.close()
				ser2.close()
				time.sleep(1)
			else:
				initialized = True
				print('Initialized magnetometers.')
				time.sleep(1)
				return [ser1, ser2]
				
				
	elif sensor_type == 'BNO055':
		initialized = False
		while initialized == False:
			# Set up serial communication with Arduino(s)
			port3 = '/dev/ttyUSB2'
			ser3 = serial.Serial(port3, 115200, timeout=2)
			ser3.flushInput()
			
			data_error = False
			try:
				result1 = collectData('BNO055', 6, ser3)
			except:
				data_error = True
				
			
			if data_error:	
				print('Failed to initialize IMU, trying again...')
				ser3.close()
				time.sleep(1)
			else:
				initialized = True
				print('Initialized IMU.')
				time.sleep(1)
				return ser3
				
				
				

def collectData_thread(main_queue, sensor_type, data_queues, ser_ID):
	### Function to continuously collect data and store in Python queue
	# main_queue:		Main Python queue
	# sensor_type:		Type of sensor; currently supported: "MLX90393", "BNO055"
	# data_queues:		Python list of Python LIFO queues to store collected data
	# ser_IDs:			Serial ID to collect data from
	
	k = 0
	end_thread = False
	
	if sensor_type=='MLX90393':
		while end_thread == False:
			# Read bytes
			ser_bytes = ser_ID.readline()
			decoded_bytes = float(ser_bytes[0:len(ser_bytes)-2].decode("utf-8"))
			
			# Decide where to store data
			if k == 0:
				data_queues[0].put(decoded_bytes)
				k = k + 1
			elif k == 1:
				data_queues[1].put(decoded_bytes)
				k = k + 1
			elif k == 2:
				data_queues[2].put(decoded_bytes)
				k = k + 1
			elif k == 3:
				data_queues[3].put(decoded_bytes)
				k = k + 1
			elif k == 4:
				data_queues[4].put(decoded_bytes)
				k = k + 1
			elif k == 5:
				data_queues[5].put(decoded_bytes)
				k = k + 1
			elif k == 6:
				data_queues[6].put(decoded_bytes)
				k = k + 1
			elif k == 7:
				data_queues[7].put(decoded_bytes)
				k = k + 1
			elif k == 8:
				data_queues[8].put(decoded_bytes)
				k = 0
				
			# Check if data collection and thread should be stopped
			dict_temp = main_queue.get()
			main_queue.put(dict_temp)
			if dict_temp['stop'] == True:
				end_thread = True
				print('Stopping data collection thread...')
				
				
	elif sensor_type=='BNO055':
		while end_thread == False:
			# Read bytes
			ser_bytes = ser_ID.readline()
			decoded_bytes = float(ser_bytes[0:len(ser_bytes)-2].decode("utf-8"))
			
			# Decide where to store data
			if k == 0:
				data_queues[0].put(decoded_bytes)
				k = k + 1
			elif k == 1:
				data_queues[1].put(decoded_bytes)
				k = k + 1
			elif k == 2:
				data_queues[2].put(decoded_bytes)
				k = k + 1
			elif k == 3:
				data_queues[3].put(decoded_bytes)
				k = k + 1
			elif k == 4:
				data_queues[4].put(decoded_bytes)
				k = k + 1
			elif k == 5:
				data_queues[5].put(decoded_bytes)
				k = 0

				
			# Check if data collection and thread should be stopped
			dict_temp = main_queue.get()
			main_queue.put(dict_temp)
			if dict_temp['stop'] == True:
				end_thread = True
				print('Stopping data collection thread...')
					
					
					
def analyzeData_thread(main_queue, data_queues1, data_queues2, data_queues3):
	### Function for collecting and analyzing data during exploratory search
	# main_queue:	Python Queue for sharing variables between threads
	# data_queues1:	Python list of Python queues to store collected magnetometer data (left tripod)
	# data_queues2:	Python list of Python queues to store collected magnetometer data (right tripod)
	# data_queues3:	Python list of Python queues to store collected IMU data
	
	end_thread = False
	k = 0
	points_mag = 15
	points_IMU = 30
	
	# Initialize dict for storing analyzed data
	data_analyzed_mag1 = dict()
	data_analyzed_mag2 = dict()
	data_analyzed_IMU = dict()
	for q in [1, 2, 3]:
		for w in ['x', 'y', 'z']:
			data_analyzed_mag1[w + str(q)] = []
			data_analyzed_mag2[w + str(q)] = []
	
	
	for w in ['x', 'y', 'z']:
		data_analyzed_IMU['angVel_' + w] = []
		data_analyzed_IMU['linAccel_' + w] = []
		
	
	# Define dicts for data array/queue pairs
	data_pairs_mag1 = {'x1': data_queues1[0], 'y1': data_queues1[1], 'z1': data_queues1[2], \
						'x2': data_queues1[3], 'y2': data_queues1[4], 'z2': data_queues1[5], \
						'x3': data_queues1[6], 'y3': data_queues1[7], 'z3': data_queues1[8]}
	data_pairs_mag2 = {'x1': data_queues2[0], 'y1': data_queues2[1], 'z1': data_queues2[2], \
						'x2': data_queues2[3], 'y2': data_queues2[4], 'z2': data_queues2[5], \
						'x3': data_queues2[6], 'y3': data_queues2[7], 'z3': data_queues2[8]}
	data_pairs_IMU = {'angVel_x': data_queues3[0], 'angVel_y': data_queues3[1], 'angVel_z': data_queues3[2], \
						'linAccel_x': data_queues3[3], 'linAccel_y': data_queues3[4], 'linAccel_z': data_queues3[5]}
						
	while end_thread == False:
		time.sleep(0.2)
		dict_temp = main_queue.get()
		main_queue.put(dict_temp)
		
		# Specify what type of analysis to do based on main queue dict
		if dict_temp['next_move'] == 'main':
			# Wait for one step of gait to finish
			time.sleep(1.512) ##################################################################### ADJUST DEPENDING ON GAIT CYCLE
			time_init = time.time()
			
			# Get data from LIFO data queues for defined number of points for each sensor type
			for m in range(points_mag):
				for q in [1, 2, 3]:
					for w in ['x', 'y', 'z']:
						data_analyzed_mag1[w + str(q)].append(data_pairs_mag1[w + str(q)].get())
						data_analyzed_mag2[w + str(q)].append(data_pairs_mag1[w + str(q)].get())
						
						
						
			for m in range(points_IMU):
				for q in ['angVel_', 'linAccel_']:
					for w in ['x', 'y', 'z']:
						data_analyzed_IMU[q + w].append(data_pairs_IMU[q + w].get())
						
						
				
				
			time_end = time.time()
			print(time_end - time_init)
			
			
		# Check if thread should be stopped
		if dict_temp['stop'] == True:
			print('Stopping analyzeData_thread')
			end_thread = True
		else:
			k = k + 1
			print(str(k) + ' passes of collectData thread\n')
			time.sleep(1.5 - (time_end - time_init))
	
		
		
	data_analyzed = [data_analyzed_mag1, data_analyzed_mag2, data_analyzed_IMU]
	return data_analyzed
