#!/bin/sh
xterm -e " cd ..; cd launch/; roslaunch turtlebot_homeworld.launch " &
sleep 5
xterm -e " cd ..; cd launch/; roslaunch amcl_home.launch " &
sleep 5
xterm -e " cd ..; cd turtlebot_interactions/turtlebot_rviz_launchers/launch/; roslaunch view_navigation.launch "
