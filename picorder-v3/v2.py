#!/usr/bin/python

from __future__ import division
import time
from datetime import datetime
import os
import socket
import commands
import math
import RPi.GPIO as GPIO
import json
import urllib
import subprocess
from smbus import SMBus
from PyComms import hmc5883l
from PyComms import mpu6050
import MHHD44780
from gps import *
import threading
import sqlite3

DEBUG=1
LOGGER=1

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Set-up SPI for analog reading 
# change these as desired - they're the pins connected from the
# SPI port on the ADC to the Cobbler
SPICLK = 11
SPIMISO = 9
SPIMOSI = 10
SPICS = 8
 
# set up the SPI interface pins
GPIO.setup(SPIMOSI, GPIO.OUT)
GPIO.setup(SPIMISO, GPIO.IN)
GPIO.setup(SPICLK, GPIO.OUT)
GPIO.setup(SPICS, GPIO.OUT)

us_trigger_pin=27
us_echo_pin=22
GPIO.setup(us_trigger_pin, GPIO.OUT)
GPIO.setup(us_echo_pin, GPIO.IN)

PIN_SWITCH=17
GPIO.setup(PIN_SWITCH, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Define analog pins
PIN_TEMP=0
PIN_VIBR=1
PIN_MICR=2
PIN_HUMD=3

# Sensor initialization
# Using i2c channel 1 - for Rev 1 Pis, change to 0
bus = SMBus(1)

mag = hmc5883l.HMC5883L()
mag.initialize()

try:
	mpu = mpu6050.MPU6050()
	mpu.dmpInitialize()
	mpu.setDMPEnabled(True)
except:
	pass

try:
	LCD=MHHD44780.HD44780()
except:
	pass


# FUNCTIONS TO READ SENSORS / DATA
def readIPaddresses():
        ips = commands.getoutput("/sbin/ifconfig | grep -i \"inet\" | grep -iv \"inet6\" | " + "awk {'print $2'} | sed -ne 's/addr\:/ /p'")
        addrs = ips.split('\n')

	return addrs

def readHostname():
	local_hostname=socket.gethostname()
	return local_hostname

# read SPI data from MCP3008 chip, 8 possible adc's (0 thru 7)
def readadc(adcnum, clockpin, mosipin, misopin, cspin):
	try:
		if ((adcnum > 7) or (adcnum < 0)):
			return -1
		GPIO.output(cspin, True)
		GPIO.output(clockpin, False)  # start clock low
		GPIO.output(cspin, False)     # bring CS low

		commandout = adcnum
		commandout |= 0x18  # start bit + single-ended bit
		commandout <<= 3    # we only need to send 5 bits here
		for i in range(5):
			if (commandout & 0x80):
				GPIO.output(mosipin, True)
			else:   
				GPIO.output(mosipin, False)
			commandout <<= 1
			GPIO.output(clockpin, True)
			GPIO.output(clockpin, False)
	 
		adcout = 0
		# read in one empty bit, one null bit and 10 ADC bits
		for i in range(12):
			GPIO.output(clockpin, True)
			GPIO.output(clockpin, False)
			adcout <<= 1
			if (GPIO.input(misopin)):
				adcout |= 0x1
	 
		GPIO.output(cspin, True)
 
		adcout /= 2	 #first bit is 'null' so drop it

	except:
		adcout = 0

	return adcout

def readTemperature():
	raw = readadc(PIN_TEMP, SPICLK, SPIMOSI, SPIMISO, SPICS)
	mv = raw * (3300.0 / 1024.0)
	tC = ((mv-100.0) / 10.0) - 40.0
	tF = (tC * 9.0 / 5.0) + 32

	return tC

def readHumidity():
	raw = readadc(PIN_HUMD, SPICLK, SPIMOSI, SPIMISO, SPICS)
	return raw

def readPCFchannel(channel):
	try:
		bus.write_byte(0x48, channel)
		# discard first value - it's the previous reading
		bus.read_byte(0x48)
		reading = bus.read_byte(0x48)

	except:
		reading = -1

	return reading

def readPCFpot():
	return readPCFchannel(1)

def readPCFtemp():
	return readPCFchannel(2)

def readPCFlight():
	raw = readPCFchannel(3)
	reading = 255 - raw
	return reading


def readHMC5883L():
	try:
		data = mag.getHeading()
		reading = {}
		reading['yaw'] = "%.0f" % data['z']
		reading['pitch'] = "%.0f" % data['y']
		reading['roll'] = "%.0f" % data['x']

	except:
		reading = {}
		reading['yaw'] = -1
		reading['pitch'] = -1
		reading['roll'] = -1

	return reading

def readMPU6050():
	# Sensor initialization

	# get expected DMP packet size for later comparison
	packetSize = mpu.dmpGetFIFOPacketSize()

	try:
		while True:
			# Get INT_STATUS byte
			mpuIntStatus = mpu.getIntStatus()
		
			if mpuIntStatus >= 2: # check for DMP data ready interrupt (this should happen frequently)
				# get current FIFO count
				fifoCount = mpu.getFIFOCount()
		
				# check for overflow (this should never happen unless our code is too inefficient)
				if fifoCount == 1024:
					# reset so we can continue cleanly
					mpu.resetFIFO()
					#print('FIFO overflow!')
		
		
				# wait for correct available data length, should be a VERY short wait
				fifoCount = mpu.getFIFOCount()
				while fifoCount < packetSize:
					fifoCount = mpu.getFIFOCount()
		
				result = mpu.getFIFOBytes(packetSize)
				q = mpu.dmpGetQuaternion(result)
				g = mpu.dmpGetGravity(q)
				ypr = mpu.dmpGetYawPitchRoll(q, g)
	
				reading = {}
				reading['yaw'] = "%.2f" % (ypr['yaw'] * 180 / math.pi)
				reading['pitch'] = "%.2f" % (ypr['pitch'] * 180 / math.pi)
				reading['roll'] = "%.2f" % (ypr['roll'] * 180 / math.pi)
		
				# track FIFO count here in case there is > 1 packet available
				# (this lets us immediately read more without waiting for an interrupt)
				fifoCount -= packetSize
				# break when you have a reading
				return reading
	except:
		reading = {}
		reading['yaw'] = -1
		reading['pitch'] = -1
		reading['roll'] = -1

		return reading
		

def read_ultrasonic():
	try:
		number_of_readings=10
		number_of_samples=10
		ping_timeout=200000
		debug = False
	
		# Timing constants
		us_trigger_pulse = 0.00001
		us_read_sleep = 0.00002
	
		all_readings = []
		for j in range(number_of_readings):
	
		        reading_list = []
			readings_used = 0
		        for i in range(number_of_samples):
				# 50 ms is the max timeout if nothing in range.
		               	# time.sleep(0.005)
				timeout_flag = False
	
		                # set our trigger high, triggering a pulse to be sent.
		                GPIO.output(us_trigger_pin, GPIO.HIGH)
		                time.sleep(us_trigger_pulse)
		                GPIO.output(us_trigger_pin, GPIO.LOW)
	
				timeout_start = datetime.now()
	
				# Wait for our pin to go high, waiting for a response.
		                while not GPIO.input(us_echo_pin):
					timeout_end = datetime.now()
					timeout_delta = timeout_end - timeout_start
					if timeout_delta.microseconds > ping_timeout:
						if debug:
							print "Timeout A"
						timeout_flag = True
						break
		                        pass
	
				# Now its high, get our start time
				timeout_start = datetime.now()
		                start = datetime.now()
		
				# wait for our input to go low
		                while GPIO.input(us_echo_pin):
					timeout_end = datetime.now()
					timeout_delta = timeout_end - timeout_start
					if timeout_delta.microseconds > ping_timeout:
						if debug:
							print "Timeout B"
						timeout_flag = True
						break
		                        pass
	
		                # Now its low, grab our end time
		                end = datetime.now()
	
		                # Store our delta.
				if not timeout_flag:
		                	delta = end - start
		   			reading_list.append(delta.microseconds)
					readings_used = readings_used + 1
	
					if debug:
						print "Microseconds %1.f" % delta.microseconds
	
		                # take a little break, it appears to help stabalise readings
		                # I suspect due to less interference with previous readings
		                time.sleep(us_read_sleep)
	
		        average_reading = sum(reading_list)/len(reading_list)
	
		        all_readings.append(average_reading)
	
		average_of_all_readings = sum(all_readings)/len(all_readings)
		average_distance=average_of_all_readings * 340
		average_distance=average_distance/20000
		return_text = "%s cm" % average_distance

	except:
		return_text = "No reading"
	
	return return_text



def get_coords():
	try:
		lat = gpsd.fix.latitude
		lon = gpsd.fix.longitude
		speed = gpsd.fix.speed
		utc = gpsd.utc
		alt = gpsd.fix.altitude

		if (math.isnan(lat)):
			lat = "No satellite fix"

		if (math.isnan(lon)):
			lon = "No satellite fix"

		if (math.isnan(speed)):
			speed = "No satellite fix"
		else:
			speed = "%s m/s" % speed

		if (utc):
			pass
		else:
			utc = "No satellite fix"

		if (math.isnan(alt)):
			alt = "No reading"
		else:
			alt = "%s metres" % alt

		sats = gpsd.satellites

		coords = [lat, lon, utc, alt, speed, sats]

	except (KeyboardInterrupt, SystemExit):
		pass

	return coords

def readVibration():
	reading = readadc(PIN_VIBR, SPICLK, SPIMOSI, SPIMISO, SPICS)

	perc_of_max = float(reading / 1024) * 100
	number_of_bars = float((16/100)) * perc_of_max
	number_of_bars = int(math.ceil(number_of_bars))
	#print 'Bars %s' % number_of_bars

	output = '|' * number_of_bars

	return output

def readSoundLevel():
	reading = readadc(PIN_MICR, SPICLK, SPIMOSI, SPIMISO, SPICS)

	print reading

	perc_of_max = float(reading / 255) * 100
	number_of_bars = float((16/100)) * perc_of_max
	number_of_bars = int(math.ceil(number_of_bars))
	#print 'Bars %s' % number_of_bars

	output = '|' * number_of_bars

	return output

def readSwitch(PIN):
	reading = GPIO.input(PIN)
	if (reading == 1):
		reading = 0
	else:
		reading = 1
	return reading


def lcdMessage(top, bottom):
	if DEBUG:
		print top
		print bottom

	msg = str(top) + "\n" + str(bottom)
	LCD.message(msg)

def buttonPressAction(channel):
	global time_stamp
	global operation
	global max_operation
	time_now = time.time()

	if (time_now - time_stamp) >= 0.3:
		if (operation == max_operation):
			operation = 0
		else:
			operation = operation + 1

		time_stamp = time_now
		print "Next!"

class GpsPoller(threading.Thread):
	def __init__(self):
		threading.Thread.__init__(self)
		global gpsd #bring it in scope
		gpsd = gps(mode=WATCH_ENABLE) #starting the stream of info
		self.current_value = None
		self.running = True #setting the thread running to true

	def run(self):
		global gpsd
		while gpsp.running:
			gpsd.next() #this will continue to loop and grab EACH set of gpsd info to clear the buffer


def currentSession():
	ts = time.time()
	st = datetime.fromtimestamp(ts).strftime('%Y%m%d%H%M%S')

	return st



if __name__ == '__main__':
	operation = 0
	max_operation = 11
	time_stamp = time.time()

	print "Using RPi.GPIO version " + GPIO.VERSION

	lcdMessage("= Picorder v2 =", "Starting up...")
	time.sleep(1)

	lcdMessage("= Picorder v2 =", "Activating GPS")
	os.system('./enable_gps.sh')
	time.sleep(3)
	gpsd = None
	gpsp = GpsPoller()
	gpsp.start()

	# Database logging
	lcdMessage("= Picorder v2 =", "Connecting to DB")
	conn = sqlite3.connect('picorder.db')
	cursor = conn.cursor()

	GPIO.add_event_detect(PIN_SWITCH, GPIO.RISING, callback=buttonPressAction)

	sessionID = currentSession()
	print 'Session ', sessionID

	# Set to 0 for normal operation, set to other number to skip to that on boot
	operation = 8
	last_operation = -1

	while True:
		now = datetime.now()
		curTime = str(now.hour) + ':' + str(now.minute) + ':' + str(now.second)

		try:
			if operation != last_operation:
				LCD=MHHD44780.HD44780()
				LCD.clear()
				last_operation = operation

			if operation == 0:
				hostname=readHostname()
				for addr in readIPaddresses():
					lcdMessage(hostname, addr)
					time.sleep(1)
	
			elif operation == 1:
				coords = get_coords()
				sqlData = coords
				sqlData.pop(5)
				sqlData.append(0)
				sqlData.append(sessionID)

				cursor.execute("INSERT INTO gpslog (lat, lon, datetime, alt, speed, uploaded, session_id) VALUES (?,?,?,?,?,?,?);", sqlData)
				conn.commit()

				lcdMessage("Lat %s" % coords[0], "Lng %s" % coords[1])
				time.sleep(2)
				lcdMessage("Alt %s" % coords[3], "Speed %s" % coords[4])
				time.sleep(2)
	
			elif operation == 2:
				lcdMessage("Potentiometer", readPCFpot())
				time.sleep(0.2)
	
			elif operation == 3:
				lcdMessage("Temperature (PCF)", readPCFtemp())
				time.sleep(0.2)
	
			elif operation == 4:
				hum = readHumidity()
				lcdMessage("Humidity", hum)
				time.sleep(2)
	
			elif operation == 5:
				lcdMessage("Light", readPCFlight())
				time.sleep(0.2)

			elif operation == 6:
				lcdMessage("Vibration", readVibration())
				time.sleep(0.25)
	
			elif operation == 7:
				lcdMessage("Sound", readSoundLevel())
				time.sleep(0.25)
	
			elif operation == 8:
				ypr = readHMC5883L()
				short_version_top = "y:" + ypr['yaw'] + " p:" + ypr['pitch']
				short_version_bot = "r:" + ypr['roll']
				lcdMessage(short_version_top, short_version_bot)
				time.sleep(0.2)
	
			elif operation == 9:
				ypr = readMPU6050()
				short_version_top = "y:" + ypr['yaw'] + " p:" + ypr['pitch']
				short_version_bot = "r:" + ypr['roll']
				lcdMessage(short_version_top, short_version_bot)
				time.sleep(0.2)
	
			elif operation == 10:
				lcdMessage("Distance", read_ultrasonic())
				time.sleep(0.5)
	
			elif operation == 11:
				lcdMessage("Temperature", readTemperature())
				time.sleep(0.2)

			#operation = operation + 1

		except KeyboardInterrupt:
			gpsp.running = False
			gpsp.join()
			GPIO.cleanup()

		except:
			pass
