import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)

GPIO.setup(11, GPIO.OUT)

GPIO.output(11, GPIO.HIGH)
time.sleep(1)
GPIO.output(11, GPIO.LOW)

