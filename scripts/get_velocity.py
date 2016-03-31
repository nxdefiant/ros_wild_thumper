#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-
#
# rostopic echo /odom/twist/twist/linear/x > /tmp/velocity.txt

import sys
import numpy as np

f = open(sys.argv[1])
l = []
for line in f:
	if "---" in line:
		continue
	l.append(float(line))

print "avg=%.2f, var=%f" % (sum(l)/len(l), np.var(l))
