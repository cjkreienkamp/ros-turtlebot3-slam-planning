cmake_minimum_required(VERSION 3.0.2)
project(rviz_demo)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  roscpp
)

catkin_package()

# if(CATKIN_ENABLE_TESTING)
#   find_package(roslaunch REQUIRED)
#   roslaunch_add_file_check(launch)
# endif()
# 
# install(DIRECTORY config launch maps urdf worlds
#         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(simple_navigation_goals src/simple_navigation_goals.cpp)
target_link_libraries(simple_navigation_goals ${catkin_LIBRARIES})
