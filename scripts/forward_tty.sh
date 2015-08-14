#!/bin/sh

socat /dev/ttyUSB0,b57600 TCP4-LISTEN:10001,reuseaddr
