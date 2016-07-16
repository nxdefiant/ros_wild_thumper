#!/bin/sh

# go
rostopic pub -1 cmd_vel geometry_msgs/Twist '[0.3, 0, 0]' '[0, 0, 0]' > /dev/null &

sleep 1

# start record
rostopic echo /odom/twist/twist/linear/x > /tmp/velocity.txt &
echoid=$!

# done record
sleep 15
kill $echoid

# stop
rostopic pub -1 cmd_vel geometry_msgs/Twist '[0, 0, 0]' '[0, 0, 0]' > /dev/null &

# print info
$(rospack find wild_thumper)/scripts/get_velocity.py /tmp/velocity.txt
