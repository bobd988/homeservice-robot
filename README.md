
[map]: ./images/nav.png "Goal"
[map1]: ./images/nav3.png "Goal reached"

# Home Service Robot

 The goal is to program a home service robot that will autonomously map an environment and navigate to pickup and deliver objects.

* Build a simulated map using Gazebo building editor
* Build a map of the environment using gmapping and teleop.
* Use Adaptive Monte Carlo Localisation to detect the robot position within the known map.
* Use the ROS move_base library to plot a path to a target pose and navigate to it.
* Write a custom node to encompass the path planning and driving libraries, listening for goal poses.
* Write a custom node to publish goal poses for the robot, then compare these to the actual pose (odometry topic) to determine success

## References
This project utilizes the following ROS packages

[gmapping](http://wiki.ros.org/gmapping)
[turtlebot_teleop](http://wiki.ros.org/turtlebot_teleop)
[turtlebot_rviz_launchers](http://wiki.ros.org/turtlebot_rviz_launchers)
[turtlebot_gazebo](http://wiki.ros.org/turtlebot_gazebo)

and make sure to install the ROS navigation stack
```
sudo apt-get install ros-kinetic-navigation
```

## Package Tree
```
    ├──                                # Official ROS packages
    |
    ├── slam_gmapping                  # gmapping_demo.launch file
    │   ├── gmapping
    │   ├── ...
    ├── turtlebot                      # keyboard_teleop.launch file
    │   ├── turtlebot_teleop
    │   ├── ...
    ├── turtlebot_interactions         # view_navigation.launch file
    │   ├── turtlebot_rviz_launchers
    │   ├── ...
    ├── turtlebot_simulator            # turtlebot_world.launch file
    │   ├── turtlebot_gazebo
    │   ├── ...
    ├──                                # customized packages 
    |
    ├── worlds                          # world files
    │   ├── ...
    ├── shellscripts                   # shell scripts files
    │   ├── ...
    ├──rviz_config                      # rviz configuration files
    │   ├── ...
    ├──wall_follower                   # wall_follower C++ node
    │   ├── src/wall_follower.cpp
    │   ├── ...
    ├──pick_objects                    # pick_objects C++ node
    │   ├── src/pick_objects.cpp
    │   ├── ...
    ├──add_markers                     # add_marker C++ node
    │   ├── src/add_markers.cpp
    │   ├── ...
    └──
```
To run the home service robot script file, open a terminal and run the following commands.

```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ catkin_make
$ cd src/shellscripts
$ ./home_service.sh
```

![alt text][map]

![alt text][map1]