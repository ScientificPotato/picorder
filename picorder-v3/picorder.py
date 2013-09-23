#!/usr/bin/python

import RPi.GPIO as GPIO
from PyComms import hmc5883l
from PyComms import mpu6050
import lcddriver
import math
import commands
import os
import time
import threading
from smbus import SMBus
from datetime import datetime
from gps import *
from Adafruit_BMP085 import BMP085










###############################################################
# LCD
def display(line_1, line_2="                    ", line_3="                    ", line_4="                    "):

	line_1 = line_1.ljust(20, " ")
	line_2 = line_2.ljust(20, " ")
	line_3 = line_3.ljust(20, " ")
	line_4 = line_4.ljust(20, " ")

	lcd.lcd_display_string(line_1, 1)
	lcd.lcd_display_string(line_2, 2)
	lcd.lcd_display_string(line_3, 3)
	lcd.lcd_display_string(line_4, 4)

	if DEBUG == 1:
		print "--------------------"
		print line_1
		print line_2
		print line_3
		print line_4
		print "--------------------"


###############################################################
# SESSION
def currentSession():
        ts = time.time()
        st = datetime.fromtimestamp(ts).strftime('%Y%m%d%H%M%S')

        return st

def readHostname():
        local_hostname=socket.gethostname()
        return local_hostname

def readIPaddresses():
        ips = commands.getoutput("/sbin/ifconfig | grep -i \"inet\" | grep -iv \"inet6\" | " + "awk {'print $2'} | sed -ne 's/addr\:/ /p'")
        addrs = ips.split('\n')

        return addrs

###############################################################
# HMC reading
def readHMC5883L():
        try:
                data = hmc.getHeading()
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

###############################################################
# MPU reading
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






###############################################################
# GPS
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






###########################################################
def readBarometer():
	try:
		temp = bmp.readTemperature()
		pressure = bmp.readPressure()
		altitude = bmp.readAltitude()

		str_temp = "Temperature:%.2f C" % temp
		str_pressure = "Pressure:%.2f hPa" % (pressure/100.0)
		str_altitude = "Altitude:%.2f m" % altitude

		reading = {}
		reading['temperature'] = str_temp
		reading['pressure'] = str_pressure
		reading['altitude'] = str_altitude

	except:
		pass

	return reading






###########################################################
def readCoordinates():
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




###############################################################
# ULTRASONIC
def readUltrasonic():
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
                                GPIO.output(US_PIN_TRIGGER, GPIO.HIGH)
                                time.sleep(us_trigger_pulse)
                                GPIO.output(US_PIN_TRIGGER, GPIO.LOW)

                                timeout_start = datetime.now()

                                # Wait for our pin to go high, waiting for a response.
                                while not GPIO.input(US_PIN_ECHO):
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
                                while GPIO.input(US_PIN_ECHO):
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

        except Exception as err:
		print type(err)
		print err.strerror
                return_text = "No reading"

        return return_text


###############################################################
# General A2D read
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

                adcout /= 2      #first bit is 'null' so drop it

        except:
                adcout = 0

        return adcout



###############################################################
# MICROPHONE/SOUND LEVEL
def readSoundLevel():
        reading = readadc(PIN_MICR, SPICLK, SPIMOSI, SPIMISO, SPICS)

#        print reading

        perc_of_max = float(reading / 255) * 100
        number_of_bars = float((16/100)) * perc_of_max
        number_of_bars = int(math.ceil(number_of_bars))
        #print 'Bars %s' % number_of_bars

        output = '#' * number_of_bars

        return output




###############################################################
# Read Humidity
def readHumidity():
        raw = readadc(PIN_HUMD, SPICLK, SPIMOSI, SPIMISO, SPICS)

	raw = str(raw)

        return raw


###############################################################
# Read MQ7
def readMQ7():
	reading = readadc(PIN_MQ7, SPICLK, SPIMOSI, SPIMISO, SPICS)

	return str(reading)

###############################################################
# Read MQ2
def readMQ2():
	reading = readadc(PIN_MQ2, SPICLK, SPIMOSI, SPIMISO, SPICS)

	return str(reading)

###############################################################
# Read Moisture
def readMoisture():
	reading = readadc(PIN_MOISTURE, SPICLK, SPIMOSI, SPIMISO, SPICS)

	return str(reading)

###############################################################
###############################################################
###############################################################
# SYS
def iterateOperation(channel):
	global operation
	global max_operation
	global time_stamp

	time_now = time.time()

	if (time_now - time_stamp) >= 0.3:
		operation = operation + 1
		time_stamp = time_now

		if DEBUG:
			print "Next operation"




###############################################################
# INIT SECTION
DEBUG=1
print "Picorder version 3"
print "Michael Horne - July 2013"
print "Using RPi.GPIO version " + GPIO.VERSION

time_stamp = time.time()
session_id = currentSession()

# GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# LCD
try:
	lcd = lcddriver.lcd()
except:
	print "LCD failed to initialise"

# ACCEL NUMBER 1
try:
	hmc = hmc5883l.HMC5883L()

except:
	print "HMC failed to initialise"

# ACCEL NUMBER 2
try:
	mpu = mpu6050.MPU6050()
	mpu.dmpInitialize()
	mpu.setDMPEnabled(True)

except:
	print "MPU failed to initialise"

# Barometer BMP085
bmp = BMP085(0x77, 0)

# Ultrasonic
US_PIN_TRIGGER=17
US_PIN_ECHO=27
GPIO.setup(US_PIN_TRIGGER, GPIO.OUT)
GPIO.setup(US_PIN_ECHO, GPIO.IN)

# Analog-to-digital converter
SPICLK = 11
SPIMISO = 9
SPIMOSI = 10
SPICS = 8

GPIO.setup(SPIMOSI, GPIO.OUT)
GPIO.setup(SPIMISO, GPIO.IN)
GPIO.setup(SPICLK, GPIO.OUT)
GPIO.setup(SPICS, GPIO.OUT)

# I2C bus
bus = SMBus(1)

# Button
PIN_SWITCH = 24
GPIO.setup(PIN_SWITCH, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Sound
PIN_MICR = 1
PIN_HUMD = 0
PIN_MQ7 = 2
PIN_MQ2 = 3
PIN_MOISTURE = 7



#############################################################
# Start-up routine

display("####################", "Picorder v3", "Starting up...", "####################")
time.sleep(0.5)

display("####################", "Picorder v3", session_id, "####################")
time.sleep(0.5)

# GPS
ENABLE_GPS = True
if ENABLE_GPS:
	display("####################", "Picorder v3", "Initialising GPS", "####################")
	os.system("./enable_gps.sh")
	time.sleep(2)
	gpsd = None
	gpsp = GpsPoller()
	gpsp.start()




###############################################################
# MAIN


if __name__ == "__main__":
#	while True:
#		reading = readadc(0, SPICLK, SPIMOSI, SPIMISO, SPICS)
#		print reading
#		reading = readadc(1, SPICLK, SPIMOSI, SPIMISO, SPICS)
#		print reading
#		reading = readadc(2, SPICLK, SPIMOSI, SPIMISO, SPICS)
#		print reading
#		reading = readadc(3, SPICLK, SPIMOSI, SPIMISO, SPICS)
#		print reading
#		reading = readadc(4, SPICLK, SPIMOSI, SPIMISO, SPICS)
#		print reading
#	exit(0)

	operation = 0
	GPIO.add_event_detect(PIN_SWITCH, GPIO.RISING, callback=iterateOperation)

	while True:
		print "Current operation: " + str(operation)
		try:
			if operation == 0:
				hostname = readHostname()
				for addr in readIPaddresses():
					display("Hostname:", hostname, "IP addresses", addr)
					time.sleep(1)

			elif operation == 1:
				now = datetime.now()
				curDate = now.strftime("%d %B %Y")
				curTime = now.strftime("%H:%M:%S")
				display("Current date is:", curDate, "Current time is:", curTime)
				time.sleep(1)

			elif operation == 2:
				coords = readCoordinates()
				display("Latitude %s" % coords[0], "Longitude %s" % coords[1], "Altitude %s" % coords[3], "Speed %s" % coords[4])
				time.sleep(0.5)

			elif operation == 3:
				mq7 = readMQ7()
				display("MQ7 sensor", "Carbon monoxide", mq7, "")
				time.sleep(0.5)

			elif operation == 4:
				ypr = readMPU6050()
				display("MPU accelerometer", "Yaw: " + ypr['yaw'], "Pitch: " + ypr['pitch'], "Roll: " + ypr['roll'])
				time.sleep(0.5)

			elif operation == 5:
				tpa = readBarometer()
				display("Barometer", tpa['temperature'], tpa['pressure'], tpa['altitude'])
				time.sleep(0.5)

			elif operation == 6:
				ultrasonic = readUltrasonic()
				display("Ultrasonic", "Distance", "Measurement", ultrasonic)
				time.sleep(0.5)

			elif operation == 7:
				level = readSoundLevel()
				display("Microphone", "Sound level", level)
				time.sleep(0.5)

			elif operation == 8:
				level = readHumidity()
				display("Humidity", "Not calibd", level)
				time.sleep(0.5)

			elif operation == 9:
				mq2 = readMQ2()
				display("MQ2 sensor", "Combustible gas", mq2, "")
				time.sleep(0.5)

			elif operation == 10:
				reading = readMoisture()
				display("Moisture", reading, "", "")
				time.sleep(0.5)

			else:
				operation = 0

		except KeyboardInterrupt:
			gpsp.running = False
			gpsp.join()
			GPIO.cleanup()
			raise

		except:
			print "Error"
			raise
