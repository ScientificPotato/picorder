#!/usr/bin/python

from Adafruit_MCP230xx import Adafruit_MCP230XX
from time import *

# Use busnum = 0 for older Raspberry Pi's (256MB)
#mcp = Adafruit_MCP230XX(busnum = 0, address = 0x20, num_gpios = 16)
# Use busnum = 1 for new Raspberry Pi's (512MB with mounting holes)
mcp = Adafruit_MCP230XX(busnum = 1, address = 0x20, num_gpios = 16)
 
# Set pins 0, 1 and 2 to output (you can set pins 0..15 this way)
OUTPUT=1
mcp.config(0, OUTPUT)
mcp.config(1, OUTPUT)
mcp.config(2, OUTPUT)
 
# Set pin 3 to input with the pullup resistor enabled
mcp.pullup(3, 1)
# Read pin 3 and display the results
print "%d: %x" % (3, mcp.input(3) >> 3)
 
# Python speed test on output 0 toggling at max speed
while (True):
  mcp.output(0, 1)  # Pin 0 High
  mcp.output(1, 0)  # Pin 0 High
  sleep(0.5)
  mcp.output(0, 0)  # Pin 1 Low
  mcp.output(1, 1)  # Pin 0 High
  sleep(0.5)

