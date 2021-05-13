#!/bin/sh
xterm -e "roslaunch my_robot turtlebot_amr.launch" &
sleep 5
xterm -e "roslaunch my_robot amcl_with_map.launch" &
sleep 5
xterm -e "roslaunch my_robot navigation_with_markers.launch" &
sleep 5
xterm -e "rosrun add_markers add_markers"