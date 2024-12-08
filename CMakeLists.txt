cmake_minimum_required(VERSION 3.0.2)
project(mundo)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  genmsg
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  tf
)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(move_robot src/moveRobot.cpp)

target_link_libraries(move_robot ${catkin_LIBRARIES})

add_executable(talker_goals src/talkerGoals.cpp)

target_link_libraries(talker_goals ${catkin_LIBRARIES})