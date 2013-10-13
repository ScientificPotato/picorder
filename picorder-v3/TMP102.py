#!/usr/bin/python

import time
from Adafruit_I2C import Adafruit_I2C

class TMP102 :
  i2c = None


  # Constructor
  def __init__(self, address=0x48, mode=1, debug=False):
    self.i2c = Adafruit_I2C(address)

    self.address = address
    self.debug = debug
    # Make sure the specified mode is in the appropriate range
    if ((mode < 0) | (mode > 3)):
      if (self.debug):
        print "Invalid Mode: Using STANDARD by default"
      self.mode = self.__BMP085_STANDARD
    else:
      self.mode = mode



  def readRawTemp(self):
    "Reads the raw (uncompensated) temperature from the sensor"
    raw = self.i2c.readU16(0x48)     #The TMP102 returns 12-bits, I think
    if (self.debug):
      print "DBG: Raw Temp: 0x%04X (%d)" % (raw & 0xFFFF, raw)
    return raw


  def readTemperature(self):
    "Gets the compensated temperature in degrees celcius"

    RawBytes = self.readRawTemp()  #get the temp from readRawTemp (above)

    temp =float(RawBytes) * (float(0.0625) / 16.0) # dividing by 16 accomplishes the bit-shift
    if (self.debug):
      print "DBG: Calibrated temperature = %f C" % temp
    
    
    return temp
