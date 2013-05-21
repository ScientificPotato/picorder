# Timing constants
E_PULSE = 0.00005
E_DELAY = 0.00005
trigger_pin=27
echo_pin=22
GPIO.setup(trigger_pin, GPIO.OUT)
GPIO.setup(echo_pin, GPIO.IN)

def read_ultrasonic():
	number_of_readings=5
	number_of_samples=15
	ping_timeout=200000
	debug = False

	all_readings = []
	for j in range(number_of_readings):

	        reading_list = []
		readings_used = 0
	        for i in range(number_of_samples):
			# 50 ms is the max timeout if nothing in range.
	               	# time.sleep(0.005)
			timeout_flag = False

			FIRMATA_PIN_RED_LED.write(1)
			FIRMATA_PIN_GREEN_LED.write(0)

	                # set our trigger high, triggering a pulse to be sent.
	                GPIO.output(trigger_pin, GPIO.HIGH)
	                time.sleep(0.00001)
	                GPIO.output(trigger_pin, GPIO.LOW)

			timeout_start = datetime.now()

			# Wait for our pin to go high, waiting for a response.
	                while not GPIO.input(echo_pin):
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
	                while GPIO.input(echo_pin):
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

			FIRMATA_PIN_RED_LED.write(0)
			FIRMATA_PIN_GREEN_LED.write(1)


	                # take a little break, it appears to help stabalise readings
	                # I suspect due to less interference with previous readings
	                time.sleep(0.00002)

	        average_reading = sum(reading_list)/len(reading_list)

	        all_readings.append(average_reading)

	FIRMATA_PIN_RED_LED.write(1)
	FIRMATA_PIN_GREEN_LED.write(1)

	average_of_all_readings = sum(all_readings)/len(all_readings)
	average_distance=average_of_all_readings * 340
	average_distance=average_distance/20000
	return_text = "%s cm" % average_distance

	FIRMATA_PIN_RED_LED.write(0)
	FIRMATA_PIN_GREEN_LED.write(0)

	return return_text
