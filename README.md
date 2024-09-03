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
- roslaunch rviz_demo core.launch world:="worlds/turtlebot3_world.world"
- roslaunch rviz_demo create_map.launch world:="worlds/turtlebot3_world.world"
- before closing rosrun map_server map_saver -f MAP_NAME
- roslaunch rviz_demo localize.launch world:="worlds/turtlebot3_world.world" map:="<path-to-ws>/src/rviz_demo/maps/MAP_NAME.yaml
- roslaunch rviz_demo planning.launch world:="worlds/turtlebot3_world.world" map:="<path-to-ws>/src/rviz_demo/maps/MAP_NAME.yaml
- run the ./bin file

How to test:
- Does the robot follow the line?
