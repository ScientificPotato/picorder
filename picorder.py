#!/usr/bin/python

import time
import os
import RPi.GPIO as GPIO
from smbus import SMBus

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
DEBUG=1
LOGGER=1

# read SPI data from MCP3008 chip, 8 possible adc's (0 thru 7)
def readadc(adcnum, clockpin, mosipin, misopin, cspin):
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
 
        adcout /= 2       # first bit is 'null' so drop it
        return adcout

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

def readTemperature():
	raw=readadc(PIN_TEMP, SPICLK, SPIMOSI, SPIMISO, SPICS)
	mv = raw * (3300.0 / 1024.0)
	tC = ((mv-100.0) / 10.0) - 40.0
	tF = (tC * 9.0 / 5.0) + 32

	return tC

# Using i2c channel 1 - for Rev 1 Pis, change to 0
bus = SMBus(1)

# Define analog pins
PIN_TEMP=0
PIN_VIBR=1
PIN_MICR=2

def readPCFchannel(channel):
	bus.write_byte(0x48, channel)
	# discard first value - it's the previous reading
	bus.read_byte(0x48)
	reading = bus.read_byte(0x48)
	return reading

def readPCFpot():
	return readPCFchannel(1)

def readPCFtemp():
	return readPCFchannel(2)

def readPCFlight():
	return readPCFchannel(3)




while True:
	print "Pot: ", readPCFpot()
	print "Temp: ", readPCFtemp()
	print "Light: ", readPCFlight()

	print "Temperature: ", readTemperature()
	print "Vibration: ", readadc(PIN_VIBR, SPICLK, SPIMOSI, SPIMISO, SPICS)
	print "Sound: ", readadc(PIN_MICR, SPICLK, SPIMOSI, SPIMISO, SPICS)
	time.sleep(0.25)
