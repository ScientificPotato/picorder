import gps
 
# Listen on port 2947 (gpsd) of localhost
print "Starting session"
session = gps.gps("localhost", "2947")

print "Starting stream watch"
session.stream(gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)
 
while True:
	try:
		print "Trying to get report"

		report = session.next()
		# Wait for a 'TPV' report and display the current time
		# To see all report data, uncomment the line below
		# print report
		if report['class'] == 'TPV':
			if hasattr(report, 'time'):
				print report.time
				print report.fix
	except KeyError:
		pass
	except KeyboardInterrupt:
		quit()
	except StopIteration:
		session = None
		print "GPSD has terminated"
