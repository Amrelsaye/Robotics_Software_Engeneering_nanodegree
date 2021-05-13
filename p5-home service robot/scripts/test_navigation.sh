#!/bin/sh
xterm -e "roslaunch my_robot turtlebot_amr.launch" &
sleep 5
xterm -e "roslaunch my_robot amcl_with_map.launch" &
sleep 5
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch"
sleep 5