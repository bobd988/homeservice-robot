#!/bin/sh
xterm -e " cd ..; cd launch/; roslaunch turtlebot_homeworld.launch " &
sleep 5
xterm -e " cd ..; cd turtlebot_simulator/turtlebot_gazebo/launch/; roslaunch gmapping_demo.launch " &
sleep 5
xterm -e " cd ..; cd turtlebot_interactions/turtlebot_rviz_launchers/launch/; roslaunch view_navigation.launch " &
sleep 5
xterm -e " cd ..; cd turtlebot/turtlebot_teleop/launch/; roslaunch keyboard_teleop.launch "
