#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-

import thread
import gps
import numpy as np
from geodesy import utm
from gps_follow_waypoints import GPSGotoCoords

DEFAULT_GPSD_HOSTNAME="wildthumper"
DEFAULT_GPSD_PORT="2948"

class GPSFollowGPSD:
	def __init__(self, hostname=DEFAULT_GPSD_HOSTNAME, port=DEFAULT_GPSD_PORT):
		self.gpsd = gps.gps(host=hostname, port=port)
		self.gpsd.stream(gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)
		self.follower = GPSGotoCoords()

	def run(self):
		coords_last = (0, 0)
		for report in self.gpsd:
			if report['class'] == "TPV":
				if report['mode'] == 0:
					print "No data from gpsd"
					continue
				coords_cur = utm.fromLatLong(report.lat, report.lon)
				coords_cur = [coords_cur.northing, coords_cur.easting]

				# Check difference to last goal
				if np.linalg.norm(np.array(coords_cur) - np.array(coords_last)) > 3:
					print "Setting new goal"
					# Set a new goal
					thread.start_new_thread(self.follower.next_pos, (report.lat, report.lon))
					coords_last = coords_cur

if __name__ == "__main__":
	p = GPSFollowGPSD()
	p.run()
