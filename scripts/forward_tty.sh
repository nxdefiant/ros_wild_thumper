#!/bin/sh
#
# Client:
# socat PTY,link=/dev/ttyS4,mode=777 TCP:wildthumper:10001

socat /dev/ttyUSB0,b57600 TCP4-LISTEN:10001,reuseaddr
