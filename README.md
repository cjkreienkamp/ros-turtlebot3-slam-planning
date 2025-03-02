# TurtleBot3 SLAM and Path Planning

This ROS Noetic package enables a TurtleBot3 robot to autonomously navigate unknown environments using SLAM for mapping and localization, and the ROS Navigation Stack for path planning.

## Features

- **Simultaneous Localization and Mapping (SLAM)**: Utilizes LIDAR data to create real-time maps of unknown environments while tracking the robot's position.
- **Autonomous Navigation**: Implements path planning algorithms to navigate from an initial position to a specified goal within the mapped environment.
- **Simulation Support**: Provides integration with Gazebo and RViz for simulation and visualization purposes.

## System Requirements

- **Operating System**: Ubuntu 20.04 LTS
- **ROS Distribution**: ROS Noetic Ninjemys
- **TurtleBot3 Packages**: Ensure that the TurtleBot3 packages are installed. Follow the instructions from the official [TurtleBot3 e-Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/).

## Installation

1. **Install ROS Noetic**:

   ```bash
   sudo apt update
   sudo apt install ros-noetic-desktop-full
   source /opt/ros/noetic/setup.bash

3. **Build a catkin workspace**:

   Clone the repository:
   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/src
   git clone https://github.com/cjkreienkamp/ros-turtlebot3-slam-planning.git
   ```
   Build a catkin workspace with a package named `rviz_demo`:
   ```bash
   mv ros-turtlebot3-slam-planning rviz_demo
   cd ~/catkin_ws
   catkin_make
   ```

2. **Launch the first program**:

   Place the following lines into `~/.bashrc`:
   ```bash
   export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/catkin_ws/src/rviz_demo/
   source ~/catkin_ws/devel/setup.bash
   ```
   Launch the program, then use `control-C` to exit:
   ```bash
   source ~/.bashrc
   cp -r ~/catkin_ws/src/rviz_demo/worlds/turtlebot3_world/ ~/.gazebo/models/turtlebot3_world
   cd ~/catkin_ws
   roslaunch rviz_demo core.launch world:="worlds/turtlebot3_world.world"
   ```
   Expected output:
   - Gazebo opens with the turtlebot3 robot and the turtlebot3 world.
   - rqt_robot_steering window opens, and you can see the robot move in the world as you change its linear and angular speed.
   - RViz opens with the turtlebot3 robot and a lidar pointcloud of red dots displaying part of the world.

## Usage

1. **Launch the Mapping Environment**:

   Open two terminals. In one of them launch the mapping program.
   ```bash
   roslaunch rviz_demo create_map.launch world:="worlds/turtlebot3_world.world"
   ```
   While visualizing the robot in RViz, drive it around in order to build a map of the environment. Keep driving until the map is complete and looks similar to the Gazebo world. When finished, save the map in the second terminal.
   ```bash
   cd ~/catkin_ws/src/rviz_demo/maps
   rosrun map_server map_saver -f map1
   ```
   Expected output:
   - Newly created `map1.yaml` and `map1.pgm` in `~/catkin_ws/src/rviz_demo/maps/`.

2. **Start SLAM**:

   Launch the localization program:
   ```bash
   roslaunch rviz_demo localize.launch world:="worlds/turtlebot3_world.world" map:="$HOME/catkin_ws/src/rviz_demo/maps/map1.yaml"
   ```
   Look at the position of the robot in Gazebo. Open RViz and use the green 2D Pose Estimate arrow to give the robot an intial estimate of its location and direction in the map. Use the rqt_robot_steering window to move the robot around until the red Pose Array arrows converge to a smaller cluster of estimations.

   Expected output:
   - Robot correctly estimates its location in RViz given the map and its sensor readings.

3. **Launch the Planning Environment**:

   Open two terminals. In one of them launch the planning program:
   ```bash
   roslaunch rviz_demo planning.launch world:="worlds/turtlebot3_world.world" map:="$HOME/catkin_ws/src/rviz_demo/maps/map1.yaml
   ```

   Localize the robot in the same way that you did in the LOCALIZATION section. Once the robot is localized, hide the Pose Array arrows by clicking the small triangle on the far left side of the RViz window to open the Displays panel and then unselect the box next to Pose Array. Stay in RViz and use the pink 2D Nav Goal arrow to give the robot a goal destination. Watch the robot as it plans out its path and then follows the line (it may skew from the path but will eventually make it to the goal) to the goal destination. Run the following line of code in the second terminal to send the robot a goal state 1 meter in front of its current position through a cpp program.
   ```bash
   cd ~
   ./catkin_ws/devel/lib/rviz_demo/simple_navigation_goals
   ```

   Expected output:
   - Robot reaches goal position when given by the user with the 2D Nav Goal arrow.
   - Robot reaches goal position when given by the simple_navigation_goals executable.

## File Structure
- `config/`: Contains configuration files for RViz and navigation parameters.
- `launch/`: Launch files to start various nodes and simulations.
- `maps/`: Directory to store saved maps.
- `src/`: Source code for custom nodes and scripts.
- `urdf/`: Unified Robot Description Format files for robot modeling.
- `worlds/`: Custom Gazebo world files for simulation.
- `CMakeLists.txt`: CMake build configuration file.
- `package.xml`: Package metadata file.

## Acknowledgments

The following repositories were used in part or fully integrated into this project.
- [ROBOTIS-GIT/turtlebot3/turtlebot3_description/urdf](https://github.com/ROBOTIS-GIT/turtlebot3/tree/master/turtlebot3_description/urdf): urdf files to create the turtlebot3_burger model of the robot
- [ROBOTIS-GIT/turtlebot3_simulations/turtlebot3_gazebo](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/master/turtlebot3_gazebo): world files to create the turtlebot3 world in Gazebo
- [dbloisi/turtlebot3_navigation_goals](https://github.com/dbloisi/turtlebot3_navigation_goals/blob/master/src/turtlebot3_navigation_goals.cpp): cpp file to send a goal location to the robot

## Summary and Conclusion

The following steps are the progression of an autonomous robot moving from an intial state to a goal state in a foreign environment.
- **Mapping**: When placed in a novel foreign environment, an autonomous robot uses knowledge from its sensors and its own movements (odometry) to create a map of the environment. In this case, the robot is a turtlebot3_burger model with a lidar sensor mounted on its top. The environment is the turtlebot3_world model that loads into Gazebo. The robot uses a ROS wrapper for OpenSlam's Gmapping. The gmapping package provides laser-based SLAM (Simultaneous Localization and Mapping). After creating the map, the robot saves it using the ROS map_server node.
- **Localization**: When placed in the environment again, the robot will compare its sensor readings with its internal map to estimate where it thinks it is located in the environment. The map is loaded into RViz using the ROS map_server node, and then the robot uses the Adaptive Monte-Carlo Localizer (amcl) to estimate its position.
- **Planning**: After knowing where it is in the environment, where it expects obstacles, and where it encounters new obstacles from its local sensor readings, the robot plans a path from an initial state to a goal state. This plan is then sent to a controller that executes the necessary joint torques to reach the goal state. The robot uses the ROS move_base node to initialize a local and a global planner, which then send desired velocities on the /cmd_vel topic to the differential drive velocity controller.

Though the robot is able to plan its path in a foreign environment given an intial position and a goal position, it does not follow the path very well. Significant time was invested to editing the path planning parameters in the following files.
- `config/base_local_planner_params.yaml`
- `config/costmap_common_params.yaml`
- `config/global_costmap_params.yaml`
- `config/local_costmap_params.yaml`

After edits, the robot was found to be able to reach its goal every time, but further improvements could not be found to follow the plan in a smooth way.
