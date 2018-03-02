#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-
#
# see https://answers.ros.org/question/174892/ros-log-file-change-of-time-stamping-format/

import os
import time
import logging
import sys
import datetime
from rosgraph.roslogging import RosStreamHandler as _RosStreamHandler, _logging_to_rospy_names, _defaultFormatter


class RosStreamHandler(_RosStreamHandler):
	def emit(self, record):
		level, color = _logging_to_rospy_names[record.levelname]
		record_message = _defaultFormatter.format(record)
		msg = os.environ.get('ROSCONSOLE_FORMAT', '[${severity}] [${time}]: ${message}')
		msg = msg.replace('${severity}', level)
		msg = msg.replace('${message}', str(record_message))
		msg = msg.replace('${walltime}', '%f' % time.time())
		msg = msg.replace('${thread}', str(record.thread))
		msg = msg.replace('${logger}', str(record.name))
		msg = msg.replace('${file}', str(record.pathname))
		msg = msg.replace('${line}', str(record.lineno))
		msg = msg.replace('${function}', str(record.funcName))
		try:
			from rospy import get_name
			node_name = get_name()
		except ImportError:
			node_name = '<unknown_node_name>'
		msg = msg.replace('${node}', node_name)
		time_str = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
		msg = msg.replace('${time}', time_str)
		msg += '\n'
		if record.levelno < logging.WARNING:
			self._write(sys.stdout, msg, color)
		else:
			self._write(sys.stderr, msg, color)
