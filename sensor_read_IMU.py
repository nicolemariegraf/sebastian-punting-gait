### Author: John Grezmak
###
### File for testing data collection with BNO055 IMU. For now assume sensor
### is connected to Ardunino Nano, connected to Raspberry Pi 3/4 with serial
### port /ttyUSB2.

import datetime
from multiprocessing.pool import ThreadPool
import pickle
import time

import matplotlib.pyplot as plt
import numpy as np
import serial

from sensor_defs import collectData, initialize_sensor

# Try to initialize IMU until successful
ser3 = initialize_sensor('BNO055')

data_length = 6*200

data1 = collectData('BNO055', data_length, ser3)

# Close all serial communication
ser3.close()

# Plot collected data
fig, ((ax1, ax2, ax3), (ax4, ax5, ax6)) = plt.subplots(2, 3, figsize=(34, 10))
srate = 20 # (Hz)
L = len(data1[0:data_length:6])
time = np.linspace(0, L*(1/srate), L)

for n, k in enumerate(['Ang Vel (rad/s)', 'Lin Accel(m/s2)']):
	for m, p in enumerate(['x', 'y', 'z']):
		eval('ax' + str(3*n + m + 1)).plot(time, data1[(3*n + m):data_length:6])
		eval('ax' + str(3*n + m + 1)).set_xlim([0, L*(1/srate)])
		eval('ax' + str(3*n + m + 1)).set_xlabel('Time (s)')
		eval('ax' + str(3*n + m + 1)).set_ylabel(k)
		eval('ax' + str(3*n + m + 1)).set_title(p)



plt.show()
	
	
