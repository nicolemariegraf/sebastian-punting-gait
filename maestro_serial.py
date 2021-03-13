### Author: John Grezmak
###
### ---Library for sending serial commands to Pololu Maestro servo 
###	controller from Raspberry Pi 4 (via USB).

import serial

class Maestro(object):
	def __init__(self, device='/dev/ttyACM0', baudrate=115200):
		self.ser = serial.Serial(device, baudrate)
		self.ser.baudrate = baudrate
		self.ser.bytesize = serial.EIGHTBITS
		self.ser.parity = serial.PARITY_NONE
		self.ser.stopbits = serial.STOPBITS_ONE
		self.ser.xonxoff = False
		self.ser.timeout = 0
		
	def get_error(self):
		# ---Check if there was an error. Returns 0 if no error, >0 if error.

		command = bytes([0xAA, 0x0C, 0xA1 & 0x7F])

		self.ser.write(command)

		data = [b'\x00', b'\x00']
		n = 0
		while n != 2:
			data[n] = self.ser.read(1)
			if not data[n]: continue
			n = n + 1

		return int.from_bytes(data[0], byteorder='big') & 0x7F + (int.from_bytes(data[1], byteorder='big') & 0x7F) << 7
		
	def get_position(self, channel):
		# ---Gets the position of a servo from a Maestro channel. Returns servo
		# position in quarter-microseconds
		#
		# channel: 	number of channel to get position information for (0, 1, ...)

		command = bytes([0xAA, 0x0C, 0x90 & 0x7F, channel])

		self.ser.write(command)

		data = [b'\x00', b'\x00']
		n = 0
		while n != 2:
			data[n] = self.ser.read(1)
			if not data[n]: continue
			n = n + 1

		return int.from_bytes(data[0], byteorder='big') + 256 * int.from_bytes(data[1], byteorder='big')
		
	def set_speed(self, channel, speed):
		# ---Sets the speed of a Maestro channel.

		# channel: 	number of channel (0, 1, ...)
		# speed: 	desired speed of channel, units (0.25us/10ms), 0 means unlimited

		command = bytes([0xAA, 0x0C, 0x87 & 0x7F, channel, speed & 0x7F, (speed >> 7) & 0x7F])
		self.ser.write(command)
		
	def set_acceleration(self, channel, accel):
		# ---Sets the acceleration of a Maestro channel. 
		#
		# channel: 	number of channel (0, 1, ...)
		# accel: 	desired acceleration of channel, units of (0.25 us)/(10 ms)/(80 ms).

		command = bytes([0xAA, 0x0C, 0x89 & 0x7F, channel, accel & 0x7F, (accel >> 7) & 0x7F])
		self.ser.write(command)
		
	def set_target(self, channel, target):
		# ---Sets the target of a Maestro channel. 
		#
		# channel: 	number of channel (0, 1, ...).
		# target: 	Desired target in quarter-microseconds.
				
		command = bytes([0xAA, 0x0C, 0x84 & 0x7F, channel, target & 0x7F, (target >> 7) & 0x7F])
		self.ser.write(command)
		
	def set_multiple_targets(self, channel, targets):
		# ---Sets the targets of contiguous block of multiple Maestro channels.
		# Note that the channels have to be contiguous, according to manual,
		# and that you only specify the channel of the first servo in the block.
		#
		# channel: First channel in contiguous block of channels to set targets for
		# targets:	Python list of targets, length equal to number of targets to set starting at first channel
		
		command = [0xAA, 0x0C, 0x1F, len(targets), channel]
		for target in targets:
			command.append(target & 0x7F)
			command.append((target >> 7) & 0x7F)
			
		command = bytes(command)
		self.ser.write(command)
		
	def close(self):
		# ---Close serial port
		
		self.ser.close()
