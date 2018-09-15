import serial 
import numpy
import matplotlib.pyplot as plt 
from drawnow import *
from time import sleep

ser = serial.Serial('COM3')

# open the line, conf UART
with ser:
	ser.setDTR(False)
	sleep(1)
	ser.flushInput()
	ser.setDTR(True)

ser = serial.Serial('COM3', baudrate=115200, timeout=None)

# set plot to animated
plt.ion()

count = 0
temp1Array = []
temp2Array = []
temp3Array = []

# flush any junk left in the serial buffer
# ser.flushInput()

def fig():
	plt.subplot(311)
	plt.plot(temp1Array, 'r-', label = 'bh')
	plt.grid('True')
	plt.xlabel("time")
	plt.legend(loc='upper left')
	# plt.ylim([-1000, 1000])

	plt.subplot(312)
	plt.plot(temp2Array, 'g-', label = 'bv')
	plt.grid('True')
	plt.xlabel("time")
	plt.legend(loc='upper left')
	# plt.ylim([-1000, 1000])

	plt.subplot(313)
	plt.plot(temp3Array, 'b-', label = 'bz')
	plt.grid('True')
	plt.xlabel("time")
	plt.legend(loc='upper left')
	# plt.ylim([-1000, 1000])
	
	# make the window move! How?!
	# plt.xlim([count-50, count])


# open the UART line
with ser:
	count = 0
	while True:
		rxString = ser.readline()

		# this might need a little bit of work later
		# need to have a good non ASCII in the string?
		if "sl".encode() in rxString:
			# print every string which is good
			tempstring = (rxString.decode('utf-8'))

			rxArray = tempstring.split()
			# print split vales in different color
			# print(rxArray[1])
			# print(rxArray[3])
			# print(rxArray[5])

			temp1 = float(rxArray[1])
			temp1Array.append(temp1)
			temp2 = float(rxArray[3])
			temp2Array.append(temp2)
			temp3 = float(rxArray[5])
			temp3Array.append(temp3)

			plt.pause(0.0001)
			count = count + 1

			if (count > 50):
				temp1Array.pop(0)
				temp2Array.pop(0)
				temp3Array.pop(0)
				plt.clf()  # clearing doesnt work yet?!
				count = 0				
			drawnow(fig)
			

			
