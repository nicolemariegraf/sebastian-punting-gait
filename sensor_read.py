### Author: John Grezmak
###
### File for testing data collection from MLX90393 sensors. For now assumes
### that there are two Arduino Nanos connected to Raspberry Pi with serial
### ports /ttyUSB0 and /ttyUSB1.

import datetime
from multiprocessing.pool import ThreadPool
import pickle
import time

import matplotlib.pyplot as plt
import numpy as np
import serial

from sensor_defs import collectData, initialize_sensor

# Try to initialize magnetometers until successful		
[ser1, ser2] = 	initialize_sensor('MLX90393')

data_length = int(1692 - 120*9)

# Start data collection threads and measure time elapsed
print('Starting data collection...')
time_init = time.time()

pool = ThreadPool(processes=2)
collectData_result1 = pool.apply_async(collectData, ('MLX90393', data_length, ser1))
collectData_result2 = pool.apply_async(collectData, ('MLX90393', data_length, ser2))
pool.close()
pool.join()

time_end = time.time()
print('Ended data collection...')
# Compute elapsed time
print('Time elasped is ' + str(time_end - time_init))

# Compute resulting average sampling rate
srate = (data_length/9)/(time_end - time_init)
print('Sampling rate is ' + str(srate))

# Retreive data from data collection threads
data = dict()
data['data1'] = collectData_result1.get()
data['data2'] = collectData_result2.get()

# Close all serial communication
ser1.close()
ser2.close()

# Plot collected data
fig, ((ax1, ax2, ax3), (ax4, ax5, ax6)) = plt.subplots(2, 3, figsize=(34, 10))
L = len(data['data1'][0:data_length:9])
time = np.linspace(0, L*(1/srate), L)

for n in range(2):
	for m in range(3):
		eval('ax' + str(3*n + m + 1)).plot(time, data['data' + str(n + 1)][3*m:data_length:9])
		eval('ax' + str(3*n + m + 1)).set_xlim([0, L*(1/srate)])
		eval('ax' + str(3*n + m + 1)).set_xlabel('Time (s)')
		eval('ax' + str(3*n + m + 1)).set_ylabel('Magnetic field strength (uT)')
		eval('ax' + str(3*n + m + 1)).set_title('Sensor ' + str(m + 1))



plt.show()

