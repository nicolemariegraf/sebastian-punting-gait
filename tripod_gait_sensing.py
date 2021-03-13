### Author: John Grezmak
###
### ---Demo for implementing tripod or inward tripod gait on Sebastian 
### robot while collecting sensor data from various sensors.

from collections import OrderedDict 
import datetime
import itertools
from multiprocessing.pool import ThreadPool
import pickle
from queue import LifoQueue, Queue
import time

import matplotlib.pyplot as plt
import numpy as np
import serial

from gait_defs import tripod_gait
from med_Sebastian_info import *
from Sebastian_library_Maestro import Sebastian
from sensor_defs import initialize_sensor, collectData_thread, analyzeData_thread

# Load PWM signal relations from file
#filename = r'/home/pi/Documents/John/Data/gait_parameters/tripod_gait_4phase_1.sav'
filename = r'/home/pi/Documents/John/Data/gait_parameters/tripod_gait_6phase_1.sav'
#filename = r'/home/pi/Documents/John/Data/gait_parameters/inward_tripod_gait_1.sav'
with open(filename, 'rb') as f:
	thetas_PWM = pickle.load(f)
	reset_config = pickle.load(f)	# Loading reset config from gait file
	

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
time.sleep(0.5)

### Set up serial communication with Arduino(s)
# Try to initialize magnetometers until successful		
[ser1, ser2] = 	initialize_sensor('MLX90393')

### Try to initialize IMU until successful
ser3 = initialize_sensor('BNO055')

### Define number of steps to take
n_steps = 25

### Create dict() to use in queue
queue_dict = dict()
queue_dict['gait_params'] = [robot, thetas_PWM, n_steps]  # list of gait parameters
queue_dict['main_gait'] = 'tripod_gait'    			# Main gait
queue_dict['next_move'] = 'main'                	# String specifying which move to make next
queue_dict['stop'] = False                      	# (Bool) If True, stop while loops in all threads

main_queue = LifoQueue()
main_queue.put(queue_dict)

### Create lists of data queues
mag_queues1 = [LifoQueue() for k in range(9)]
mag_queues2 = [LifoQueue() for k in range(9)]
IMU_queues = [LifoQueue() for k in range(6)]

### Start movement and data collection
print('Start threads...\n')
pool = ThreadPool(processes=4)
err1 = pool.apply_async(robot.threading_main, (main_queue, tripod_gait, n_steps))
err2 = pool.apply_async(collectData_thread, (main_queue, 'MLX90393', mag_queues1, ser1))
err3 = pool.apply_async(collectData_thread, (main_queue, 'MLX90393', mag_queues2, ser2))
err4 = pool.apply_async(collectData_thread, (main_queue, 'BNO055', IMU_queues, ser3))
#data_analyzed = pool.apply_async(analyzeData_thread, (main_queue, mag_queues1, mag_queues2, IMU_queues))
pool.close()
pool.join()

# Check for errors
if not err1.get():
    err1.get()
    
if not err2.get():
    err2.get()
    
if not err3.get():
    err3.get()
    
if not err4.get():
    err4.get()  
    
      
#data_mag1 = data_analyzed.get()[0]
#data_mag2 = data_analyzed.get()[1]
#data_IMU = data_analyzed.get()[2]

data_mag1 = dict()
data_mag2 = dict()
data_IMU = dict()

for q in [1, 2, 3]:
	for w in ['x', 'y', 'z']:
		data_mag1[w + str(q)] = []
		data_mag2[w + str(q)] = []


for w in ['x', 'y', 'z']:
	data_IMU['angVel_' + w] = []
	data_IMU['linAccel_' + w] = []
	

# Define dicts for data array/queue pairs
data_pairs_mag1 = {'x1': mag_queues1[0], 'y1': mag_queues1[1], 'z1': mag_queues1[2], \
					'x2': mag_queues1[3], 'y2': mag_queues1[4], 'z2': mag_queues1[5], \
					'x3': mag_queues1[6], 'y3': mag_queues1[7], 'z3': mag_queues1[8]}
data_pairs_mag2 = {'x1': mag_queues2[0], 'y1': mag_queues2[1], 'z1': mag_queues2[2], \
					'x2': mag_queues2[3], 'y2': mag_queues2[4], 'z2': mag_queues2[5], \
					'x3': mag_queues2[6], 'y3': mag_queues2[7], 'z3': mag_queues2[8]}
data_pairs_IMU = {'angVel_x': IMU_queues[0], 'angVel_y': IMU_queues[1], 'angVel_z': IMU_queues[2], \
					'linAccel_x': IMU_queues[3], 'linAccel_y': IMU_queues[4], 'linAccel_z': IMU_queues[5]}

# Get data from LIFO data queues for defined number of points for each sensor type
mag_length = min(list(itertools.chain([k.qsize() for k in mag_queues1], [k.qsize() for k in mag_queues2])))
IMU_length = min([k.qsize() for k in IMU_queues])
for m in range(mag_length):
	for q in [1, 2, 3]:
		for w in ['x', 'y', 'z']:
			data_mag1[w + str(q)].append(data_pairs_mag1[w + str(q)].get())
			data_mag2[w + str(q)].append(data_pairs_mag2[w + str(q)].get())
			
			
			
for m in range(IMU_length):
	for q in ['angVel_', 'linAccel_']:
		for w in ['x', 'y', 'z']:
			data_IMU[q + w].append(data_pairs_IMU[q + w].get())
			
			
			
			
# Close all serial communication
error = robot.mu.get_error()
if error:
	print(error)

robot.mu.close()
ser1.close()
ser2.close()
ser3.close()

# Save data to file
directions = ['north', 'east', 'south', 'west']
direction = directions[2]

gait_types = ['straight_stance_', 'inward_stance_']
gait_type = gait_types[0]

terrains = ['dry_sand_', 'wet_sand_', 'linoleum_']
terrain = terrains[0]

trial = 3
#file_logging = r'/home/pi/Documents/John/Data/11_27_20/' + gait_type + direction + terrain + 'trial' + str(trial) + '.sav'
file_logging = r'/home/pi/Documents/John/Data/12_23_20/' + gait_type + direction + terrain + 'trial' + str(trial) + '.sav'
#file_logging = r'/home/pi/Documents/John/Data/temp.sav'
f = open(file_logging, 'w')
f.close()

with open(file_logging, 'wb') as f:
	pickle.dump(data_mag1, f)
	pickle.dump(data_mag2, f)
	pickle.dump(data_IMU, f)


# Plot data
plot_results = True

if plot_results:
	plot_type = 'mag'
	#plot_type = 'IMU'
	
	if plot_type == 'mag':
		fig, ((ax1, ax2, ax3), (ax4, ax5, ax6)) = plt.subplots(2, 3, figsize=(20, 20))
		t = np.linspace(0, mag_length/10.9, mag_length)
		ax1.plot(t, data_mag1['x1'][-1::-1])
		ax2.plot(t, data_mag1['x2'][-1::-1])
		ax3.plot(t, data_mag1['x3'][-1::-1])
		
		ax4.plot(t, data_mag2['x1'][-1::-1])
		ax5.plot(t, data_mag2['x2'][-1::-1])
		ax6.plot(t, data_mag2['x3'][-1::-1])
	elif plot_type == 'IMU':
		fig, ((ax1, ax2, ax3), (ax4, ax5, ax6)) = plt.subplots(2, 3, figsize=(20, 20))
		t = np.linspace(0, IMU_length/20, IMU_length)
		ax1.plot(t, data_IMU['linAccel_x'][-1::-1])
		ax2.plot(t, data_IMU['linAccel_y'][-1::-1])
		ax3.plot(t, data_IMU['linAccel_z'][-1::-1])
		
		ax4.plot(t, data_IMU['angVel_x'][-1::-1])
		ax5.plot(t, data_IMU['angVel_y'][-1::-1])
		ax6.plot(t, data_IMU['angVel_z'][-1::-1])
		

	plt.show()
