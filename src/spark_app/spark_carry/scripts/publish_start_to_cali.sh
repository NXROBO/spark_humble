#!/bin/bash

#BASEPATH=$(cd `dirname $0`; pwd)
gnome-terminal -x bash -c "ros2 run spark_carry send_topic.sh"


#gnome-terminal --title="spark_control" --geometry 34x10+63+305 -- bash -c "ros2 run spark_teleop spark_teleop_node 0.14 0.5"
