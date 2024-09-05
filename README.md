# rviz-demo



### Title

RViz Demo<br/>
Sapienza Università di Roma<br/>
Robot Programming 2023-24



### Abstract

This program controls a mobile robot in ROS and displays simple systems. The program is able to show:
- a map (received from the map server)
- laser scans
- mobile bases (as a circle)
All items are displayed according to their transform
- particles of localization
 The program is able to issue
 issue /initialpose message, to initialize the localizer
 and /move_base/goal messages to set a planner destination



### Introduction

The following steps are the progression of an autonomous robot moving from an intial state to a goal state in a foreign environment.
- MAPPING: When placed in a novel foreign environment, an autonomous robot uses knowledge from its sensors and its own movements (odometry) to create a map of the environment. In this case, the robot is a turtlebot3_burger model with a lidar sensor mounted on its top. The environment is the turtlebot3_world model that loads into Gazebo. The robot uses a ROS wrapper for OpenSlam's Gmapping. The gmapping package provides laser-based SLAM (Simultaneous Localization and Mapping). After creating the map, the robot saves it using the ROS map_server node.
- LOCALIZATION: When placed in the environment again, the robot will compare its sensor readings with its internal map to estimate where it thinks it is located in the environment. The map is loaded into RViz using the ROS map_server node, and then the robot uses the Adaptive Monte-Carlo Localizer (amcl) to estimate its position.
- PLANNING: After knowing where it is in the environment, where it expects obstacles, and where it encounters new obstacles from its local sensor readings, the robot plans a path from an initial state to a goal state. This plan is then sent to a controller that executes the necessary joint torques to reach the goal state. The robot uses the ROS move_base node to initialize a local and a global planner, which then send desired velocities on the /cmd_vel topic to the differential drive velocity controller.



### Literature Review

The following Git repositories were used in part or fully integrated into this project.
- [ROBOTIS-GIT/turtlebot3/turtlebot3_description/urdf](https://github.com/ROBOTIS-GIT/turtlebot3/tree/master/turtlebot3_description/urdf) --> turtlebot3_burger urdf files to create the model of the robot
- [ROBOTIS-GIT/turtlebot3_simulations/turtlebot3_gazebo](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/master/turtlebot3_gazebo) --> turtlebot3 world files to create the world in Gazebo
- [dbloisi/turtlebot3_navigation_goals](https://github.com/dbloisi/turtlebot3_navigation_goals/blob/master/src/turtlebot3_navigation_goals.cpp) --> cpp file to send a goal location to the robot



### Methods and Results

The following section is divided into the basic parts of the program (initialization, mapping, localization, planning). It describes how to compile, run, and test. Later steps can only be accomplished if prior steps were completed successfully.

1) INITIALIZATION: download code, compile with catkin, view robot and world in Gazebo, view robot and lidar sensor data in RViz, make the robot move

```
mkdir -p catkin_ws/src
roslaunch rviz_demo core.launch world:="worlds/turtlebot3_world.world"
```

2) MAPPING

3) LOCALIZATION

4) PLANNING



### Discussion
OPPORTUNITIES FOR IMPROVEMENT AND PROBLEMS

### Conclusion
SUMMARY OF WHAT I ACHIEVED

### References and Appendices

How to compile:
- create catkin_ws
- download into src folder
- download dependencies
- place the following two lines of code at the bottom of ~/.bashrc
- export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:<path-to-ws>/src/rviz_demo/
- source <path-to-ws>/devel/setup.bash
- source ~/.bashrc
- catkin_make

Goals to run and test:
- displaying the robot (in Gazebo + RViz), world (in Gazebo), and robot's perception of the world through lidar (RViz), and making the robot move
- putting the robot in an unkown world and collecting lidar data as the robot moves to create an internal map of the world
- localizing the robot in the world by using amcl and its pre-built map
- sending a desired goal position to the robot, which it must navigate in the world using only the map and lidar sensor data

How to run:
- roslaunch rviz_demo create_map.launch world:="worlds/turtlebot3_world.world"
- before closing rosrun map_server map_saver -f MAP_NAME
- roslaunch rviz_demo localize.launch world:="worlds/turtlebot3_world.world" map:="<path-to-ws>/src/rviz_demo/maps/MAP_NAME.yaml
- roslaunch rviz_demo planning.launch world:="worlds/turtlebot3_world.world" map:="<path-to-ws>/src/rviz_demo/maps/MAP_NAME.yaml
- run the ./bin file

How to test:
- Does the robot follow the line?
